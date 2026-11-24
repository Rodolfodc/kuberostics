terraform {
  required_providers {
    kubernetes = {
      source = "hashicorp/kubernetes"
    }
  }
}

locals {
  detector_image = var.detector_image != "" ? var.detector_image : "vacuum-detector:${var.ros_distro}"
  node_port      = coalesce(var.node_port, var.transport == "bridge" ? 30448 : 30447)

  # Manifests live in deploy/k8s as templates: Terraform applies exactly what scripts/check-cloud.sh validates with kubeconform.
  detector_docs = templatefile("${path.module}/../../../deploy/k8s/detector.yaml.tpl", {
    namespace               = var.namespace
    transport               = var.transport
    ros_distro              = var.ros_distro
    detector_image          = local.detector_image
    image_pull_policy       = var.image_pull_policy
    bridge_image            = var.bridge_image
    bridge_config           = file("${path.module}/../../../deploy/bridge/bridge-cloud.json5")
    model_file              = var.model_file
    weights_url             = var.weights_url
    curl_image              = var.curl_image
    weights_volume_size     = var.weights_volume_size
    node_port               = local.node_port
    detector_cpu_request    = var.detector_cpu_request
    detector_memory_request = var.detector_memory_request
    detector_memory_limit   = var.detector_memory_limit
  })

  cloud_sync_docs = var.enable_cloud_sync ? templatefile("${path.module}/../../../deploy/k8s/cloud-sync.yaml.tpl", {
    namespace                  = var.namespace
    mosquitto_image            = var.mosquitto_image
    minio_image                = var.minio_image
    mosquitto_secret_name      = var.mosquitto_secret_name
    minio_secret_name          = var.minio_secret_name
    policy_weights_volume_size = var.policy_weights_volume_size
  }) : ""

  # "kind/name" -> decoded manifest
  manifests = {
    for doc in compact(split("\n---\n", "${local.detector_docs}\n---\n${local.cloud_sync_docs}")) :
    "${yamldecode(doc).kind}/${yamldecode(doc).metadata.name}" => yamldecode(doc)
    if trimspace(doc) != ""
  }
}

resource "kubernetes_namespace_v1" "this" {
  metadata {
    name = var.namespace
  }

  lifecycle {
    precondition {
      condition     = var.transport == "bridge" || var.allow_unfiltered_rmw_zenoh
      error_message = "transport = rmw_zenoh has no ACL here and violates 009a-R19 (only the two perception topics cross). Set allow_unfiltered_rmw_zenoh = true only for a measurement."
    }
  }
}

resource "kubernetes_manifest" "this" {
  for_each   = local.manifests
  manifest   = each.value
  depends_on = [kubernetes_namespace_v1.this]
}

# Root module for the local K3s cluster (k3d "kuberostics"). Run only through tools/tf.sh (fixed directory, fixed kubeconfig).
# apply/destroy need the user's confirmation. State is local and gitignored (*.tfstate*); never put secrets in variables files.
terraform {
  required_version = ">= 1.9"
  required_providers {
    kubernetes = {
      source  = "hashicorp/kubernetes"
      version = "~> 2.35"
    }
  }
}

# No config_path here on purpose: tools/tf.sh exports KUBE_CONFIG_PATH and KUBE_CTX (the project's own kubeconfig and the
# k3d-kuberostics context), so Terraform can never reach the user's own kubeconfig or another cluster.
provider "kubernetes" {}

module "workloads" {
  source = "./modules/workloads"

  ros_distro  = var.ros_distro
  transport   = var.transport
  weights_url = var.weights_url
  node_port   = var.node_port

  allow_unfiltered_rmw_zenoh = var.allow_unfiltered_rmw_zenoh
  enable_cloud_sync          = var.enable_cloud_sync
}

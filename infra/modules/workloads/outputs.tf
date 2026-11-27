output "transport_node_port" {
  value = local.node_port
}

# Rendered manifests, for scripts/check-cloud.sh (kubeconform) and review; no cluster needed to evaluate them.
output "manifests" {
  value = local.manifests
}

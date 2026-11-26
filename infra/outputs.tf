output "transport_node_port" {
  description = "NodePort of the cluster side of the transport; the robot connects to tcp/<k3d node IP>:<this port>."
  value       = module.workloads.transport_node_port
}

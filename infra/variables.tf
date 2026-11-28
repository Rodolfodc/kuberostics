variable "ros_distro" {
  description = "ROS distribution of the detector image (vacuum-detector:<distro>); one cluster run uses one distro (Art. X)."
  type        = string
  default     = "jazzy"
}

variable "transport" {
  description = "Robot-cloud transport (Q-06). ASSUMPTION: the project owner has NOT confirmed Q-06; 'bridge' (zenoh-bridge-ros2dds, R19 allow-list) is the recommendation of the transport benchmark. Switch with -var transport=rmw_zenoh."
  type        = string
  default     = "bridge"
}

variable "allow_unfiltered_rmw_zenoh" {
  description = "rmw_zenoh has no ACL in this module (it does not meet 009a-R19). Refused unless this is true; for measurements only."
  type        = bool
  default     = false

  validation {
    condition     = var.transport == "bridge" || var.allow_unfiltered_rmw_zenoh
    error_message = "transport = rmw_zenoh has no ACL here and violates 009a-R19 (only the two perception topics cross). Set allow_unfiltered_rmw_zenoh = true only for a measurement."
  }
}

variable "node_port" {
  description = "NodePort of the transport Service. Null: 30448 for bridge, 30447 for rmw_zenoh."
  type        = number
  default     = null
}

variable "weights_url" {
  description = "Optional URL the detector Pod downloads yolo weights from when the weights volume is empty. Empty: supply the weights by hand (see infra/README.md). Not a secret: do not embed credentials in it."
  type        = string
  default     = ""
}

variable "model_file" {
  description = "Weights file name in the weights volume (yolo11x.pt in the cloud; yolo11n.pt for a CPU-only local cluster that cannot meet the age limits with yolo11x)."
  type        = string
  default     = "yolo11x.pt"
}

variable "enable_cloud_sync" {
  description = "Deploy the MQTT broker and the policy-weights store (scaffold of the cloud cycle). Needs the Secrets documented in infra/README.md."
  type        = bool
  default     = false
}

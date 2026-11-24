variable "ros_distro" {
  description = "ROS distribution of the detector image (vacuum-detector:<distro>); one cluster run uses one distro (Art. X)."
  type        = string
  default     = "jazzy"
}

variable "transport" {
  description = "Robot-cloud transport (Q-06). ASSUMPTION: the project owner has NOT confirmed Q-06; 'bridge' (zenoh-bridge-ros2dds, R19 allow-list) is the recommendation of the T902 measurement. Switch with -var transport=rmw_zenoh."
  type        = string
  default     = "bridge"
}

variable "weights_url" {
  description = "Optional URL the detector Pod downloads yolo weights from when the weights volume is empty. Empty: supply the weights by hand (see infra/README.md). Not a secret: do not embed credentials in it."
  type        = string
  default     = ""
}

variable "enable_cloud_sync" {
  description = "Deploy the MQTT broker and the policy-weights store (scaffold of the cloud cycle). Needs the Secrets documented in infra/README.md."
  type        = bool
  default     = false
}

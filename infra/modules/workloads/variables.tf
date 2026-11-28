variable "namespace" {
  type    = string
  default = "vacuum-cloud"
}

variable "ros_distro" {
  type = string
  validation {
    condition     = contains(["jazzy", "lyrical"], var.ros_distro)
    error_message = "ros_distro must be jazzy or lyrical."
  }
}

variable "transport" {
  description = "Q-06, not yet confirmed by the project owner. 'bridge' = zenoh-bridge-ros2dds sidecar with the R19 allow-list; 'rmw_zenoh' = rmw_zenohd router in the Pod."
  type        = string
  default     = "bridge"
  validation {
    condition     = contains(["bridge", "rmw_zenoh"], var.transport)
    error_message = "transport must be bridge or rmw_zenoh."
  }
}

variable "allow_unfiltered_rmw_zenoh" {
  description = "rmw_zenoh has no ACL in this module, so it does not meet 009a-R19. The module refuses it unless this is true (explicit, temporary, for measurements only)."
  type        = bool
  default     = false
}

variable "detector_image" {
  description = "Empty: vacuum-detector:<ros_distro>, imported into k3d with `k3d image import` (never pulled)."
  type        = string
  default     = ""
}

variable "image_pull_policy" {
  type    = string
  default = "Never"
}

variable "bridge_image" {
  description = "Pinned by digest (same as scripts/transport-bench.sh)."
  type        = string
  default     = "eclipse/zenoh-bridge-ros2dds:1.10.2@sha256:0183b7e83790a8c57a069743776c2d913041760eb19e64d8cc4fbf7374269034"
}

variable "model_file" {
  description = "Weights file name inside the weights volume (not committed, not in the image). yolo11x in the cloud."
  type        = string
  default     = "yolo11x.pt"
}

variable "weights_url" {
  type    = string
  default = ""
}

variable "curl_image" {
  type    = string
  default = "curlimages/curl:8.11.1"
}

variable "weights_volume_size" {
  type    = string
  default = "1Gi"
}

variable "node_port" {
  description = "NodePort of the transport: 30448 for the bridge, 30447 for rmw_zenoh (as in the transport benchmark). Empty: derived from transport."
  type        = number
  default     = null
}

variable "detector_cpu_request" {
  type    = string
  default = "1"
}

variable "detector_cpu_limit" {
  type    = string
  default = "4"
}

variable "detector_memory_request" {
  type    = string
  default = "2Gi"
}

variable "detector_memory_limit" {
  type    = string
  default = "4Gi"
}

variable "enable_cloud_sync" {
  description = "Scaffold of the cloud cycle: Mosquitto and the policy-weights store (MinIO)."
  type        = bool
  default     = false
}

variable "mosquitto_image" {
  type    = string
  default = "eclipse-mosquitto:2.0.20"
}

variable "minio_image" {
  type    = string
  default = "minio/minio:RELEASE.2025-04-22T22-12-26Z"
}

variable "mosquitto_secret_name" {
  description = "Name of a Secret (key passwd) created out of band; its content is never in git or in Terraform state."
  type        = string
  default     = "mosquitto-passwd"
}

variable "minio_secret_name" {
  description = "Name of a Secret (keys root-user, root-password) created out of band."
  type        = string
  default     = "policy-store-credentials"
}

variable "policy_weights_volume_size" {
  type    = string
  default = "2Gi"
}

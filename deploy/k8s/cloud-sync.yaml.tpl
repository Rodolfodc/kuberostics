# MQTT broker and policy-weights store (docs/architecture.md section 4, cloud cycle). Scaffold only: ingestion, training Job,
# digital twin and dashboard are not here (blocked on spec 008 / D-11). Disabled unless enable_cloud_sync = true.
# Credentials come from Secrets created out of band (never in git, never in Terraform state); see infra/README.md.
apiVersion: v1
kind: ConfigMap
metadata:
  name: mosquitto-config
  namespace: ${namespace}
data:
  mosquitto.conf: |
    listener 1883
    allow_anonymous false
    password_file /mosquitto/secret/passwd
    persistence true
    persistence_location /mosquitto/data/
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: mosquitto
  namespace: ${namespace}
  labels: {app: mosquitto}
spec:
  replicas: 1
  strategy: {type: Recreate}
  selector:
    matchLabels: {app: mosquitto}
  template:
    metadata:
      labels: {app: mosquitto}
    spec:
      automountServiceAccountToken: false
      securityContext: {runAsNonRoot: true, runAsUser: 1883, fsGroup: 1883, seccompProfile: {type: RuntimeDefault}}
      containers:
        - name: mosquitto
          image: ${mosquitto_image}
          ports: [{containerPort: 1883}]
          resources:
            requests: {cpu: 50m, memory: 32Mi}
            limits: {cpu: 500m, memory: 128Mi}
          securityContext: {allowPrivilegeEscalation: false, readOnlyRootFilesystem: true, capabilities: {drop: [ALL]}}
          livenessProbe:
            tcpSocket: {port: 1883}
            initialDelaySeconds: 5
            periodSeconds: 15
          volumeMounts:
            - {name: config, mountPath: /mosquitto/config, readOnly: true}
            - {name: secret, mountPath: /mosquitto/secret, readOnly: true}
            - {name: data, mountPath: /mosquitto/data}
      volumes:
        - {name: config, configMap: {name: mosquitto-config}}
        - {name: secret, secret: {secretName: ${mosquitto_secret_name}, defaultMode: 288}}   # 288 = 0440; key: passwd (mosquitto_passwd file)
        - {name: data, emptyDir: {}}
---
apiVersion: v1
kind: Service
metadata:
  name: mosquitto
  namespace: ${namespace}
spec:
  selector: {app: mosquitto}
  ports: [{name: mqtt, port: 1883, targetPort: 1883}]
---
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: policy-weights
  namespace: ${namespace}
spec:
  accessModes: [ReadWriteOnce]
  resources:
    requests:
      storage: ${policy_weights_volume_size}
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: policy-store
  namespace: ${namespace}
  labels: {app: policy-store}
spec:
  replicas: 1
  strategy: {type: Recreate}
  selector:
    matchLabels: {app: policy-store}
  template:
    metadata:
      labels: {app: policy-store}
    spec:
      automountServiceAccountToken: false
      securityContext: {runAsNonRoot: true, runAsUser: 10001, fsGroup: 10001, seccompProfile: {type: RuntimeDefault}}
      containers:
        - name: minio   # versioned bucket with ETag: the robot downloads with If-None-Match (200 or 304)
          image: ${minio_image}
          args: ["server", "/data", "--console-address", ":9001", "--config-dir", "/tmp/minio-config"]
          env:
            - name: MINIO_ROOT_USER
              valueFrom: {secretKeyRef: {name: ${minio_secret_name}, key: root-user}}
            - name: MINIO_ROOT_PASSWORD
              valueFrom: {secretKeyRef: {name: ${minio_secret_name}, key: root-password}}
          ports: [{containerPort: 9000}]
          resources:
            requests: {cpu: 100m, memory: 256Mi}
            limits: {cpu: "1", memory: 1Gi}
          securityContext: {allowPrivilegeEscalation: false, readOnlyRootFilesystem: true, capabilities: {drop: [ALL]}}
          livenessProbe:
            httpGet: {path: /minio/health/live, port: 9000}
            initialDelaySeconds: 10
            periodSeconds: 15
          volumeMounts:
            - {name: data, mountPath: /data}
            - {name: tmp, mountPath: /tmp}
      volumes:
        - {name: data, persistentVolumeClaim: {claimName: policy-weights}}
        - {name: tmp, emptyDir: {}}
---
apiVersion: v1
kind: Service
metadata:
  name: policy-store
  namespace: ${namespace}
spec:
  selector: {app: policy-store}
  ports: [{name: s3, port: 9000, targetPort: 9000}]

# Cambio relevante frente a Docker v3

El archivo README principal de esta versión (v4) reemplaza la guía v3:
- Windows + Docker Desktop + PowerShell, sin Ubuntu WSL adicional.
- `ping` instalado permanentemente en Dockerfile.
- Zenoh opcional (`--profile pi`), ROS_DOMAIN_ID=42 en Docker y 30 en Pi.
- Scripts ejecutables con saltos de línea LF y normalización en Dockerfile.
- El antiguo `compose.override.yaml` NO forma parte de esta versión.
  Si en tu copia local existe `docker/compose.override.yaml` de pruebas anteriores,
  elimínalo o renómbralo antes de usar v4 para evitar servicios duplicados o montajes conflictivos.

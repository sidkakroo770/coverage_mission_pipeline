#!/usr/bin/env bash
set -e
WORKSPACE="${1:-$HOME/coverage_ws}"
TARGET="$HOME/.local/bin"
mkdir -p "$TARGET"
cat > "$TARGET/coverage-swarm" <<SH
#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
exec ros2 run coverage_mission_pipeline coverage-swarm "\$@"
SH
cat > "$TARGET/coverage-swarm-check" <<SH
#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
exec "$WORKSPACE/src/coverage_mission_pipeline/scripts/check_environment.sh" "\$@"
SH
chmod +x "$TARGET/coverage-swarm" "$TARGET/coverage-swarm-check"
printf 'Installed %s and %s\n' "$TARGET/coverage-swarm" "$TARGET/coverage-swarm-check"

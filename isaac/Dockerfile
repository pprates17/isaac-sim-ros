FROM nvcr.io/nvidia/isaac-sim:5.1.0

USER root

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        liblttng-ust1 \
        liblttng-ust-dev && \
    rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

# Create a wrapper script for python3 that uses Isaac Sim’s internal Python (3.11)
RUN cat <<'EOF' >/usr/local/bin/python3
#!/bin/bash
# Wrapper to use Isaac Sim's internal Python
DIR=/isaac-sim
exec "$DIR/python.sh" "$@"
EOF
RUN chmod +x /usr/local/bin/python3

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER 1234:1234

# Override default entrypoint
ENTRYPOINT ["/entrypoint.sh"]

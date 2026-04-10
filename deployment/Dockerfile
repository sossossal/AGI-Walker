FROM python:3.9-slim

WORKDIR /app

# Set PYTHONPATH and pip defaults
ENV PYTHONPATH=/app \
    PIP_DEFAULT_TIMEOUT=300 \
    PIP_RETRIES=10 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# Keep pip current so it can resolve modern wheels where available.
RUN python -m pip install --upgrade pip setuptools wheel

# Install Zenoh and ML libs
# Note: Zenoh Python wrapper is 'eclipse-zenoh'
RUN pip install eclipse-zenoh numpy torch gymnasium psutil

# Copy application code
COPY . /app

CMD ["python", "-m", "agi_walker.core.distributed.run_learner"]

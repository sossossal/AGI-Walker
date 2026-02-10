FROM python:3.9-slim

WORKDIR /app

# Install system dependencies
# gcc/python3-dev might be needed for some pip packages
RUN apt-get update && apt-get install -y \
    gcc \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Zenoh and ML libs
# Note: Zenoh Python wrapper is 'eclipse-zenoh'
RUN pip install eclipse-zenoh numpy torch gymnasium

# Copy application code
COPY . /app

# Set PYTHONPATH
ENV PYTHONPATH=/app

CMD ["python", "distributed/run_learner.py"]

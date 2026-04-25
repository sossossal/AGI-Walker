FROM python:3.9-slim

WORKDIR /app

# Set PYTHONPATH and pip defaults
ENV PYTHONPATH=/app \
    PIP_DEFAULT_TIMEOUT=300 \
    PIP_RETRIES=10 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# Keep pip current so it can resolve modern wheels where available.
RUN python -m pip install --upgrade pip setuptools wheel

# Install the baseline distributed/runtime dependencies directly in the image.
# Keep PyYAML aligned with the package baseline so skills_loader imports fail
# during docker build instead of later during actor discovery.
RUN pip install eclipse-zenoh numpy torch gymnasium psutil pyyaml

# Copy application code
COPY . /app

RUN python -c "import yaml; from agi_walker.skills_loader import SkillsLoader; print('distributed_base_import_ok')"

CMD ["python", "-m", "agi_walker.core.distributed.run_learner"]

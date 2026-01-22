#!/bin/bash
# Deploy Flock You to Raspberry Pi

PI_HOST="admin@flocktracker-pi"
PI_PATH="/home/admin/flock-you"

echo "Deploying Flock You to $PI_HOST..."

# Sync api directory (excluding venv, pycache, data)
rsync -avz --progress \
    --exclude 'venv/' \
    --exclude '__pycache__/' \
    --exclude '*.pyc' \
    --exclude 'data/' \
    --exclude 'exports/' \
    api/ "$PI_HOST:$PI_PATH/api/"

# Sync service file if needed
rsync -avz --progress \
    flockyou.service "$PI_HOST:$PI_PATH/"

echo ""
echo "Deploy complete!"
echo ""
echo "To restart the service on the Pi, run:"
echo "  ssh $PI_HOST 'sudo systemctl restart flockyou'"

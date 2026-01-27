#!/bin/bash
# Deploy Flock You to Raspberry Pi

PI_HOST="admin@flocktracker-pi"
PI_PATH="/home/admin/flock-you"

echo "Deploying Flock You to $PI_HOST..."
echo ""

# Sync .env file from project root
echo "Syncing .env configuration..."
rsync -avz --progress \
    .env "$PI_HOST:$PI_PATH/"

# Sync api directory (excluding venv, pycache, data, exports)
echo ""
echo "Syncing api directory..."
rsync -avz --progress \
    --exclude 'venv/' \
    --exclude '__pycache__/' \
    --exclude '*.pyc' \
    --exclude 'data/' \
    --exclude 'exports/' \
    api/ "$PI_HOST:$PI_PATH/api/"

# Sync service file
echo ""
echo "Syncing service file..."
rsync -avz --progress \
    flockyou.service "$PI_HOST:$PI_PATH/"

# Install/update dependencies on Pi
echo ""
echo "Installing dependencies on Pi..."
ssh "$PI_HOST" "cd $PI_PATH/api && \
    python3 -m venv venv 2>/dev/null || true && \
    source venv/bin/activate && \
    pip install --upgrade pip && \
    pip install -r requirements.txt"

echo ""
echo "=========================================="
echo "Deploy complete!"
echo "=========================================="
echo ""

# Ask to restart service
read -p "Restart flockyou service now? [Y/n] " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]] || [[ -z $REPLY ]]; then
    echo "Restarting flockyou service..."
    ssh "$PI_HOST" "sudo systemctl restart flockyou"
    echo "Service restarted."
    echo ""
    echo "To view logs:"
    echo "  ssh $PI_HOST 'sudo journalctl -u flockyou -f'"
else
    echo "Skipped. To restart manually:"
    echo "  ssh $PI_HOST 'sudo systemctl restart flockyou'"
fi
echo ""

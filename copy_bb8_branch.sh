#!/bin/bash

# Script to copy main branch from embedded-purdue/BB8 to chaudhary-nikhil/E22_Senior_Design
# Creates a new branch called bno055-aakash

set -e  # Exit on any error

echo "🚀 Starting branch copy process..."

# Check if git is available
if ! command -v git &> /dev/null; then
    echo "❌ Git is not installed. Please install Git first."
    exit 1
fi

# Create temporary directory
TEMP_DIR=$(mktemp -d)
echo "📁 Using temporary directory: $TEMP_DIR"

# Clone source repository
echo "📥 Cloning source repository (embedded-purdue/BB8)..."
cd "$TEMP_DIR"
git clone https://github.com/embedded-purdue/BB8.git
cd BB8

# Ensure we're on main branch
echo "🔄 Ensuring we're on main branch..."
git checkout main

# Clone destination repository
echo "📥 Cloning destination repository (chaudhary-nikhil/E22_Senior_Design)..."
cd "$TEMP_DIR"
git clone https://github.com/chaudhary-nikhil/E22_Senior_Design.git
cd E22_Senior_Design

# Create and switch to new branch
echo "🌿 Creating new branch: bno055-aakash..."
git checkout -b bno055-aakash

# Copy all files from source (excluding .git directory)
echo "📋 Copying files from embedded-purdue/BB8 main branch..."
rsync -av --exclude='.git' ../BB8/ .

# Add all changes
echo "➕ Adding files to git..."
git add .

# Commit the changes
echo "💾 Committing changes..."
git commit -m "Copy main branch from embedded-purdue/BB8 to bno055-aakash

This commit copies the complete main branch from embedded-purdue/BB8 repository
to create a new branch bno055-aakash in E22_Senior_Design repository.

Source: https://github.com/embedded-purdue/BB8/tree/main
Destination: https://github.com/chaudhary-nikhil/E22_Senior_Design/tree/bno055-aakash

Components copied:
- components/ (bus_i2c, imu_bno055, imu_mpu6050, serial_stream)
- main/ (app_main.c and CMakeLists.txt)
- scripts/ (Python visualization scripts)
- Configuration files (CMakeLists.txt, sdkconfig.defaults, etc.)"

# Push the new branch
echo "🚀 Pushing new branch to GitHub..."
git push origin bno055-aakash

# Cleanup
echo "🧹 Cleaning up temporary files..."
cd /
rm -rf "$TEMP_DIR"

echo "✅ Successfully copied embedded-purdue/BB8 main branch to bno055-aakash!"
echo "🔗 New branch URL: https://github.com/chaudhary-nikhil/E22_Senior_Design/tree/bno055-aakash"
echo ""
echo "📝 Next steps:"
echo "   1. Verify the branch was created correctly on GitHub"
echo "   2. Test the code in your local environment"
echo "   3. Make any necessary adjustments for your specific setup"
echo "   4. Consider merging this branch into main if everything works correctly"



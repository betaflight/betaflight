#!/bin/bash

# Ensure the .git/hooks directory exists
if [ ! -d ".git/hooks" ]; then
  echo "No .git/hooks directory found. Are you inside a Git repository?"
  exit 1
fi

# Create a symbolic link for the pre-commit hook
ln -sf ../../pre-commit .git/hooks/pre-commit

echo "Pre-commit hook installed successfully."

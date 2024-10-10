#!/bin/bash

# Ensure the .git/hooks directory exists
if [ ! -d ".git/hooks" ]; then
  echo "No .git/hooks directory found. Are you inside a Git repository?"
  exit 1
fi

# Check if the pre-commit hook exists
if [ -L ".git/hooks/pre-commit" ]; then
  # Remove the symbolic link for the pre-commit hook
  rm .git/hooks/pre-commit
  echo "Pre-commit hook uninstalled successfully."
else
  echo "No pre-commit hook installed."
fi

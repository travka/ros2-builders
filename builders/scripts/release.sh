#!/bin/bash
set -e

# Release automation script

VERSION=$1
if [ -z "$VERSION" ]; then
    echo "Usage: $0 <version>"
    echo "Example: $0 1.0.0"
    exit 1
fi

echo "Creating release v${VERSION}..."

# Update CHANGELOG
sed -i.bak "/## \[Unreleased\]/a\\
\\
## [${VERSION}] - $(date +%Y-%m-%d)
" CHANGELOG.md
rm CHANGELOG.md.bak

# Commit changes
git add CHANGELOG.md
git commit -m "chore: prepare release v${VERSION}"

# Tag release
git tag -a "v${VERSION}" -m "Release v${VERSION}"

# Push
git push origin main
git push origin "v${VERSION}"

echo "Release v${VERSION} created and pushed!"

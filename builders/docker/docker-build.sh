#!/bin/bash
set -e

IMAGE_NAME="travka/ros2-humble"
ROS_DISTRO="humble"
PLATFORMS="linux/amd64,linux/arm64"

usage() {
    echo "Usage: $0 [--platform <platform>] [--push] [--tag <tag>]"
    echo "  --platform   Target platform (default: $PLATFORMS)"
    echo "  --push       Push to Docker Hub"
    echo "  --tag        Image tag (default: latest)"
    exit 1
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --platform)
            PLATFORMS="$2"
            shift 2
            ;;
        --push)
            PUSH=true
            shift
            ;;
        --tag)
            TAG="$2"
            shift 2
            ;;
        *)
            usage
            ;;
    esac
done

TAG=${TAG:-latest}

echo "Building ${IMAGE_NAME}:${TAG} for ${PLATFORMS}"

BUILD_ARGS=(
    --platform "$PLATFORMS"
    --build-arg ROS_DISTRO=$ROS_DISTRO
    -t "${IMAGE_NAME}:${TAG}"
)

if [ "$PUSH" = true ]; then
    BUILD_ARGS+=(--push)
    echo "Pushing to Docker Hub..."
else
    BUILD_ARGS+=(--load)
    echo "Loading to local Docker..."
fi

docker buildx build "${BUILD_ARGS[@]}" -f Dockerfile .

if [ "$PUSH" = true ]; then
    echo "Pushed ${IMAGE_NAME}:${TAG}"
fi

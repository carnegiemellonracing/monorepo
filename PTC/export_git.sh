#!/bin/bash

HASH=$(git rev-parse --short HEAD)
COMMIT_CLEAN=$(git describe --tags --always HEAD)
COMMIT_DIRTY=$(git describe --dirty --tags --always)
IS_DIRTY=1

if [[ "$COMMIT_CLEAN" == "$COMMIT_DIRTY" ]]; then
  IS_DIRTY=0
fi

echo "const uint32_t GIT_INFO = 0x$HASH; const uint8_t IS_UNCOMMITTED = $IS_DIRTY;" > ./Inc/gitcommit.h

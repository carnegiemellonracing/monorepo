#!/bin/bash

git fetch origin main:main
git clang-format-13 main --style=file

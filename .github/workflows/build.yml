name: Build
run-name: ${{ github.actor }}-${{github.repository}} Build 🚀

on: pull_request

jobs:
  build:
    runs-on: self-hosted
    steps:
      - name: Checkout Project
        uses: actions/checkout@v4.2.2
        with:
          submodules: true

      - name: Build Project
        uses: threeal/cmake-action@v2.1.0
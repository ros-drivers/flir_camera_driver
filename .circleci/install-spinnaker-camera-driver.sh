#!/bin/bash

install-spinnaker-camera-driver() {
  set -e
  local LSB_RELEASE=$(lsb_release -cs)
  local ARCH=$(dpkg --print-architecture)
  local SPINNAKER_VERSION="1.13.0.31"
  local SPINNAKER_HEADER_DIR=/usr/include/spinnaker
  local SPINNAKER_ARCHIVE_DIR="/tmp/spinnaker-${SPINNAKER_VERSION}-${ARCH}.tar.gz"
  local SPINNAKER_INSTALLER_DIR="/tmp/spinnaker-${SPINNAKER_VERSION}-${ARCH}"
  local SPINNAKER_RULE_FILE=/etc/udev/rules.d/40-flir-spinnaker.rules
  local SPINNAKER_DOWNLOAD_URL_VAR="SPINNAKER_DOWNLOAD_URL_${SPINNAKER_VERSION//./_}_${LSB_RELEASE}_${ARCH}"
  local SPINNAKER_DOWNLOAD_URL="${!SPINNAKER_DOWNLOAD_URL_VAR}"

  if [ -e $SPINNAKER_HEADER_DIR/Spinnaker.h ]; then
    echo "Spinnaker camera driver is already installed"
    return 0
  fi

  echo "Downloading driver $SPINNAKER_VERSION for $LSB_RELEASE $ARCH"

  if [ ! -e $SPINNAKER_ARCHIVE_DIR ]; then
    if [ "$(which gdown)" = "" ]; then
      pip install -U gdown
    fi

    gdown -q "$SPINNAKER_DOWNLOAD_URL" -O $SPINNAKER_ARCHIVE_DIR

    if [ ! -e $SPINNAKER_ARCHIVE_DIR ]; then
      echo "Failed to download spinnaker driver: $SPINNAKER_DOWNLOAD_URL"
      return 1
    fi
  else
    echo "Using already downloaded archive"
  fi

  if [ ! -d $SPINNAKER_INSTALLER_DIR ]; then
    echo "Unarchiving"

    (cd /tmp && tar zxvf $SPINNAKER_ARCHIVE_DIR)

    if [ ! -d $SPINNAKER_INSTALLER_DIR ]; then
      echo "No directory found in $SPINNAKER_INSTALLER_DIR. Unarchiving failed?"
      return 1
    fi
  else
    echo "Using already unarchived installer"
  fi

  if [ "$(which gdebi)" = "" ]; then
    apt install -q -qq -y gdebi
  fi

  echo "Installing"
  cd $SPINNAKER_INSTALLER_DIR
  for i in {0..3}; do
    if dpkg -i libspin*.deb; then break; fi
    apt-get install -f -qq -y
  done
  for i in {0..3}; do
    if dpkg -i spinview-qt-*.deb; then break; fi
    apt-get install -f -qq -y
  done
  for i in {0..3}; do
    if dpkg -i spinupdate-*.deb; then break; fi
    apt-get install -f -qq -y
  done
  for i in {0..3}; do
    if dpkg -i spinnaker-*.deb; then break; fi
    apt-get install -f -qq -y
  done

  echo "Installing udev rules"

  if [ ! -e $SPINNAKER_RULE_FILE ]; then
    mkdir -p /etc/udev/rules.d/
    echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1e10", MODE:="0666"' > $SPINNAKER_RULE_FILE

    systemctl restart udev || true
  else
    echo "udev rule file is already installed"
  fi

  echo "Done"

  return 0
}

install-spinnaker-camera-driver

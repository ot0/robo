# ロボットアームの制御実験

ROS2やHIDデバイスの操作などを動作させる実験

## 概要

対象: [MR-999CP](https://www.elekit.co.jp/product/MR-999CP) 生産完了品

## install

### 前提条件

uvがインストールされていること。
[uvのインストール](https://docs.astral.sh/uv/getting-started/installation/)

### Windows11の場合

```sh
uv sync
```


### WSLの場合

#### USBを接続する手順
https://learn.microsoft.com/ja-jp/windows/wsl/connect-usb

USBをbindするため、管理者権限が必要です。
{busid}は、listから選んでください。

```powershell
usbipd list
usbipd bind -i 12ed:1003
usbipd attach --wsl -i 12ed:1003
```

必要なライブラリのインストールとUSBデバイスを一般権限で動作できるように設定します。

```sh
sudo apt-get install libusb-1.0-0-dev libudev-dev
uv sync
sudo vi 99-mr999.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

99-mr999.rules

```text
SUBSYSTEM=="usb", ATTR{idVendor}=="12ed", ATTR{idProduct}=="1003", MODE="0664", GROUP="plugdev"
```

上記の設定で一般権限で動作できるように設定していない場合、ルート権限が必要です。

```sh
sudo -E ~/.local/bin/uv run main.py
```
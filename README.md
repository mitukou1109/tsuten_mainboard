# tsuten_mainboard

通天メイン基板用リポジトリ

制御用ROSメタパッケージ：https://github.com/mitukou1109/tsuten

インターフェース用ROSパッケージ：https://github.com/mitukou1109/tsuten_real_robot

### WSLにUSBデバイスを認識させるとき

1. usbipdを[Windows側](https://github.com/dorssel/usbipd-win)と[WSL側](https://github.com/dorssel/usbipd-win/wiki/WSL-support)にインストール
1. Powershellで`usbipd wsl list` → USBデバイスのBUSIDを取得
1. `usbipd wsl attach -a -b <BUSID> -d Ubuntu-20.04`

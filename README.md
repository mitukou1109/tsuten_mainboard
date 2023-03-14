# tsuten_mainboard

通天メイン基板用リポジトリ

制御用ROSメタパッケージ：https://github.com/mitukou1109/tsuten

### WSLにST-Linkを認識させるとき

1. usbipdをWindows側（https://github.com/dorssel/usbipd-win）とWSL側（https://github.com/dorssel/usbipd-win/wiki/WSL-support）にインストール
1. Powershellで`usbipd wsl list` → ST-LinkのBUSIDを取得
1. `usbipd wsl attach -a -b 2-3 -d Ubuntu-20.04`

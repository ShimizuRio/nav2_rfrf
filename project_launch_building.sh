#! /usr/bin/bash +x

# source /home/dstudent/nav2_rfrf/.acsl/bashrc
# echo ${PROJECT}${TARGET}${x86}
# echo ${ROS_DOMAIN_ID}

# cd /home/dstudent/nav2_rfrf/.acsl/0_host_commands/scripts/

# --- MODIFIED: ここからが汎用化のための重要な変更 ---
# このスクリプトファイル自身の場所を基準に、プロジェクトのルートディレクトリを動的に決定します。
# これにより、このリポジトリをどこにクローンしても、パスが正しく解決されるようになります。
PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# --- 研究室の共通環境設定を読み込む ---
# 以前のハードコードされたパスの代わりに、動的に決定したパスを使用します。
source "${PROJECT_ROOT}/.acsl/bashrc"
echo "--- Project: ${PROJECT}${TARGET}${x86}, ROS_DOMAIN_ID: ${ROS_DOMAIN_ID} ---"
echo "--- Project Root found at: ${PROJECT_ROOT} ---"

# --- 必須コンテナ群（ドライバなど）を起動 ---
# dupコマンドが置かれているディレクトリに移動
cd "${PROJECT_ROOT}/.acsl/0_host_commands/scripts"

#dup vl53l1x
dup switchbot
dup rf_building

#! /usr/bin/bash +x

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

echo "--- Starting essential driver & service containers... ---"
dup microros
dup rf_tf

# rplidarの起動
# docker-compose.ymlへのパスも動的に解決します。
TAG=rplidar${x86} CONTAINER_NAME=rplidar_front COMPOSE_PROJECT_NAME=rplidar_front_rf ROS_LAUNCH=launch_rplidar.sh LARGS=front HOSTNAME=$(hostname | sed -e 's/-/_/g') docker compose -f "${PROJECT_ROOT}/.acsl/4_docker/docker-compose.yml" up common -d
TAG=rplidar${x86} CONTAINER_NAME=rplidar_back COMPOSE_PROJECT_NAME=rplidar_back_rf ROS_LAUNCH=launch_rplidar.sh LARGS=back HOSTNAME=$(hostname | sed -e 's/-/_/g') docker compose -f "${PROJECT_ROOT}/.acsl/4_docker/docker-compose.yml" up common -d


# --- あなた専用のNav2環境を起動 ---
# 古いコンテナは起動しないようにコメントアウト
dup rf_robot
# dup slam_toolbox localization bld10_4F

echo ""
echo "#####################################################"
echo "### Starting Custom Nav2 (AMCL) Environment...    ###"
echo "#####################################################"

# Nav2用のdocker-compose.ymlがあるディレクトリへのパスも動的に解決します。
NAV2_COMPOSE_DIR="${PROJECT_ROOT}/3_dockerfiles_robot"

if [ -f "$NAV2_COMPOSE_DIR/docker-compose.nav2.yml" ]; then
    # Nav2コンテナをビルド＆起動する。
    # 中のプログラム(Nav2とrobot_node)は、DockerfileのCMD命令が自動で起動してくれる。
    (cd "$NAV2_COMPOSE_DIR" && docker compose -f docker-compose.nav2.yml up -d --build)
    
    echo ""
    echo "##############################################"
    echo "### Nav2 System launch command sent.       ###"
    echo "##############################################"
else
    echo "[ERROR] Custom Nav2 compose file not found at: $NAV2_COMPOSE_DIR/docker-compose.nav2.yml"
fi

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
dup slam_toolbox localization bld10_4F

echo ""
echo "#####################################################"
echo "### Starting Custom Nav2 (AMCL) Environment...    ###"
echo "#####################################################"

# Nav2用のdocker-compose.ymlがあるディレクトリへのパスも動的に解決します。
NAV2_COMPOSE_DIR="${PROJECT_ROOT}/3_dockerfiles_robot"
NAV2_IMAGE_NAME="kasekiguchi/acsl-common:nav2_demo_x86"

# Nav2イメージが未作成なら自動でビルドする
if ! docker images "${NAV2_IMAGE_NAME}" | grep nav2_demo_x86 > /dev/null; then
    echo "[INFO] nav2_demo_x86 image not found. Building now..."
    docker build -t ${NAV2_IMAGE_NAME} -f "${NAV2_COMPOSE_DIR}/dockerfile.nav2" "${NAV2_COMPOSE_DIR}"
else
    echo "[INFO] nav2_demo_x86 image already exists. Skipping build."
fi

# 通常のdocker-composeで起動（--build は不要）
if [ -f "$NAV2_COMPOSE_DIR/docker-compose.nav2.yml" ]; then
    (cd "$NAV2_COMPOSE_DIR" && docker compose -f docker-compose.nav2.yml up -d)
    
    echo ""
    echo "##############################################"
    echo "### Nav2 System launch command sent.       ###"
    echo "##############################################"
else
    echo "[ERROR] Custom Nav2 compose file not found at: $NAV2_COMPOSE_DIR/docker-compose.nav2.yml"
fi
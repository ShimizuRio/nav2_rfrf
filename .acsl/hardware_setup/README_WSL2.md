# Set up WSL2 

<!-- @import "[TOC]" {cmd="toc" depthFrom=1 depthTo=6 orderedList=false} -->

## docker のインストール

RP image install ＝＞ssh でログイン後

基本的に[こちら](https://kinsta.com/jp/blog/install-docker-ubuntu/)と[こちら](https://www.kagoya.jp/howto/cloud/container/dockerubuntu/)に従う。
以下まとめ（まとめてコピペで良い）

```bash
sudo apt update
sudo apt install -y ca-certificates curl gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io
sudo usermod -aG docker $USER
su - ${USER}
```

## [setupに戻る](../README.md#setup)
ifneq (,$(wildcard ./.env))
    include .env
    export
endif

task := ${shell grep -A0 'task_name:' cfg/main.yaml | tail -n1 | awk '{ print $$2}'}
IP_ADDRESS := $(shell hostname -I | cut -d' ' -f1)

.PHONY: all

build:
	docker build \
	--build-arg BASE_IMAGE=nvidia/cuda:13.0.2-base-ubuntu24.04 \
	--build-arg ARCH=amd64 \
	-t hainingluo/unitree_dev:latest \
	-f Dockerfile .

build-jetson:
	docker build \
	--build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-base:r36.2.0 \
	--build-arg ARCH=arm64 \
	-t hainingluo/unitree_dev:latest \
	-f Dockerfile .

compile:
	docker container stop unitree_dev | true && docker container rm unitree_dev | true
	docker run \
		-it \
		-e DISPLAY \
    	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v /dev:/dev \
		-v ${PWD}/:/unitree_dev:rw \
		--detach \
		--privileged \
		--runtime nvidia \
		--gpus all \
		--network host \
		--name unitree_dev \
		hainingluo/unitree_dev
# 	docker exec -it unitree_dev bash -c "git config --global --add safe.directory /unitree_dev"
# 	docker exec -it unitree_dev bash -c "cd /unitree_dev && python3 -m pip install -e ."
# 	docker exec -it unitree_dev bash -c "cd /unitree_dev/third_party/act && python3 -m pip install -e ."
	docker container stop unitree_dev

run:
	docker start unitree_dev
	sleep 1
	docker exec -it unitree_dev bash -c "source /unitree_ros2/setup.sh && export DISPLAY=:0 && bash"
	docker container stop unitree_dev

debug:
	xhost +si:localuser:root >> /dev/null
	docker start unitree_dev
	sleep 1
	docker exec -it unitree_dev bash -c "source /opt/ros/humble/setup.sh && bash"

stop:
	docker container stop unitree_dev

monitor-gpu:
	@docker start unitree_dev
	@docker exec -it unitree_dev bash -c "watch -n 1 nvidia-smi"
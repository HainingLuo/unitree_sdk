ifneq (,$(wildcard ./.env))
    include .env
    export
endif

task := ${shell grep -A0 'task_name:' cfg/main.yaml | tail -n1 | awk '{ print $$2}'}
IP_ADDRESS := $(shell hostname -I | cut -d' ' -f1)

.PHONY: all

build:
	docker build -t hainingluo/unitree_sdk:latest .

install-dynamixel:
	git submodule update --init --recursive
	docker start unitree_sdk
	sleep 1
	docker exec -it unitree_sdk bash -c "pip install -e third_party/DynamixelSDK/python"

compile:
	docker container stop unitree_sdk | true && docker container rm unitree_sdk | true
	docker run \
		-it \
		-e ROS_IP="${ROS_IP}" \
		-e ROS_MASTER_URI="${ROS_MASTER_URI}" \
		-e DISPLAY \
    	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v /dev:/dev \
		-v ${PWD}/:/unitree_dev:rw \
		--detach \
		--privileged \
		--runtime nvidia \
		--gpus all \
		--network host \
		--name unitree_sdk \
		hainingluo/unitree_sdk
# 	docker exec -it unitree_sdk bash -c "git config --global --add safe.directory /unitree_sdk"
# 	docker exec -it unitree_sdk bash -c "cd /unitree_sdk && python3 -m pip install -e ."
# 	docker exec -it unitree_sdk bash -c "cd /unitree_sdk/third_party/act && python3 -m pip install -e ."
	docker container stop unitree_sdk

run:
	docker start unitree_sdk
	sleep 1
	docker exec -it unitree_sdk bash -c "source /opt/ros/noetic/setup.bash && roscore"
	docker container stop unitree_sdk

run-local:
	docker start unitree_sdk
	sleep 1
	docker exec -it unitree_sdk bash -c "source /opt/ros/noetic/setup.bash && \
			export ROS_IP=127.0.0.1 && \
			export ROS_MASTER_URI="http://127.0.0.1:11311" && roscore"
	docker container stop unitree_sdk

run-local-new:
	docker start unitree_sdk
	sleep 1
	docker exec -it unitree_sdk bash -c "source /opt/ros/noetic/setup.bash && \
			export ROS_IP=10.101.119.100 && \
			export ROS_MASTER_URI="http://10.101.119.100:11311" && roscore"
	docker container stop unitree_sdk

debug:
	docker start unitree_sdk
	sleep 1
	docker exec -it unitree_sdk bash -c "bash"

debug-local:
	# xhost +si:localuser:root >> /dev/null
	docker start unitree_sdk
	sleep 1
	docker exec -it unitree_sdk bash -c "source /opt/ros/noetic/setup.bash && \
			export ROS_IP=127.0.0.1 &&  \
			export ROS_MASTER_URI="http://127.0.0.1:11311" && bash"

stop:
	docker container stop unitree_sdk

rqt-image-view:
	xhost +si:localuser:root >> /dev/null
	docker start unitree_sdk
	docker exec -it unitree_sdk bash -c "source /opt/ros/noetic/setup.bash && export DISPLAY=:15 && rqt --perspective-file cfg/rqt/Default.perspective"	

monitor-tensorboard:
	@docker exec -it unitree_sdk bash -c "ls data/$$task/results/" && \
	docker start unitree_sdk && \
	docker exec -it unitree_sdk bash -c "tensorboard --logdir=/unitree_sdk/data/$$task/results/ --port=6006 --host=$$IP_ADDRESS"

monitor-gpu:
	@docker start unitree_sdk
	@docker exec -it unitree_sdk bash -c "watch -n 1 nvidia-smi"
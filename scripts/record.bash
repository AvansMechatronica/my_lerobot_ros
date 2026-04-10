lerobot-record \
    --robot.type=lerobot_robot_ur \
    --robot.cameras="{ front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}}" \
    --display_data=true \
    --dataset.num_episodes=5 \
    --dataset.single_task="Grab the black cube" \
    --dataset.streaming_encoding=true \
    --teleop.type=lerobot_teleoperator_teachbot \
    --dataset.vcodec=auto \
    --dataset.push_to_hub=False \
    --dataset.repo_id=./record-test 
    # --dataset.vcodec=auto \
    #--dataset.encoder_threads=2


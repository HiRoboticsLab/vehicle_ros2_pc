date=$(date +%Y-%m-%d)
_time=$(date +%H:%M:%S)

second=$(date +%s -d $_time)
offset=$((2))

add=$(($second+$offset))
time=$(date +%H:%M:%S -d "1970-01-01 UTC $add seconds")
nanosecond=$(date +%N)

echo "$date"
echo "$_time"
echo "$time"
echo "$nanosecond"

ros2 topic pub /vehicle/adjust_date --once std_msgs/msg/String data:\ \'${date}\ ${time}.$nanosecond\'\ 
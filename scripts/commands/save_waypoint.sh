NAME=$1
X=$2
Y=$3


rosservice call /dodobot/db_waypoints/save_pose "{
    name: $NAME, waypoint: {
        header: {frame_id: map},
        pose: {
            position: {x: $X, y: $Y: z: 0.0},
            orientation: {w: 1.0}
        }
    }
}"

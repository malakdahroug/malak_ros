target = 0
upper_limit = 1.7
lower_limit = 0

if ball_position_1 is not None:
    if lower_limit < ball_position_1.transform.translation.x < upper_limit:
        target = ball_position_1.transform.translation

if ball_position_2 is not None:
    if lower_limit < ball_position_2.transform.translation.x < upper_limit:
        target = ball_position_2.transform.translation

if ball_position_3 is not None:
    if lower_limit < ball_position_3.transform.translation.x < upper_limit:
        target = ball_position_3.transform.translation

if ball_position_4 is not None:
    if lower_limit < ball_position_4.transform.translation.x < upper_limit:
        target = ball_position_4.transform.translation

if not target == 0:
    adj = abs(target.x - current_amcl.position.x)
    opp = abs(target.y - current_amcl.position.y)
    hyp = math.sqrt(adj ** 2 + opp ** 2)

    angle = cos(adj / hyp)

    if target.x >= current_amcl_position.x:
        qw = cos(angle / 2)
        qz = sin(angle / 2)
    else:
        qw = cos(angle / 2)
        qz = -sin(angle / 2)

    auto_pose_1 = [target.x, target.y, qz, qw]

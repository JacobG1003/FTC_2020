public class sudoAuto {
    /*
    fSpeed              Forward
    hSpeed              Horizontal
    tSpeed              Turn
    sensorDistance1     2m sensor1
    sensorDistance2     2m sensor2
    sensorColor         color sensor in front
    sensorColorGround   color sensor facing ground

    while time < 3
        hSpeed = 1
    hSpeed = 0

    while sensorDistance1 > 3
        fSpeed = 1
    fSpeed = 0

    while sensorDistance1 > sensorDistance2
        tSpeed = 0.1
    while sensorDistance1 < sensorDistance2
        tSpeed = -0.1
    tSpeed = 0

    while(sensorColor.red > 100 && sensorColor.blue > 100 %% sensorColor.green < 100)
        hSpeed = 0.5
    hSpeed = 0

    code to grab block

    while time < 1
        fSpeed = -1

    if team = red
        while (colorSensorGround.red < 150 && blue > 100 && green > 100 )
           hSpeed = -1
           timer.start
        timer.stop
    else if team = blue
        while (colorSensorGround.blue < 150 && red > 100 && green > 100 )
           hSpeed = -1
           timer.start
        timer.stop
    hSpeed = 0

    drop block

    while time < timer
        hSpeed = 1
    hSpeed - 0

    while sensorDistance1 > 3
        fSpeed = 1
    fSpeed = 0

    while sensorDistance1 > sensorDistance2
        tSpeed = 0.1
    while sensorDistance1 < sensorDistance2
        tSpeed = -0.1
    tSpeed = 0

    while(sensorColor.red > 100 && sensorColor.blue > 100 %% sensorColor.green < 100)
        hSpeed = 0.5
    hSpeed = 0
    code to grab block

    while time < 1
        fSpeed = -1
    fSpeed = 0

    if team = red
        while (colorSensorGround.red < 150 && blue > 100 && green > 100 )
           hSpeed = -1
       timer.stop
    else if team = blue
        while (colorSensorGround.blue < 150 && red > 100 && green > 100 )
           hSpeed = -1
    hSpeed = 0

    code to drop block


    Drive Sideways to brick side
    Look for bricks
    Drive up to bricks
    get parallel to bricks
    drive sideways searching for a non yellow brick then stop
    pick it up
    back up
    drive sideways to bridge
    drop block once past
    repeat 1-9
    stop on top of line
    * */
}

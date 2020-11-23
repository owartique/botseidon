#!/usr/bin/python3
# -*- coding: utf-8 -*

print("Content-type: text/html; charset=utf-8\n")

html = """<!DOCTYPE html><base href="http://10.3.141.1:8888/"><link rel="stylesheet" type="text/css" media="screen" href="WebRessources/style.css" />
<head>
    <title>LELME2002</title>
</head>
    <centered>
        Welcome to the Minibot project of the ELEME2002 class :)

        <ul>
            <li>I'm White Spirit, please take care of me !</li>
            <li>Please do not interchange the chips on my tower/motor PWM boards !</li>
            <li>Try to respect the C-file interface when programming me because
            it will be the same in the robotic project (Q2) !</li>
            <li>As a piece of advice: encapsulate C-code, this will be much faster to be executed !</li>
            <li>Before any tests on the motors, be sure that my wheels are not
            on the ground !</li>
        </ul>

        <form action="" method="post">
            <input type="submit" value="Magic button">
        </form>

</centered>
</html>
"""

print(html)

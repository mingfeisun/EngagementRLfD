from naoqi import ALProxy


animation_player = ALProxy("ALAnimationPlayer", "192.168.1.101", 9559)
animation_player.runTag("enthusiastic")

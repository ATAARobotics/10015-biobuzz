 # Hyper Droid 10015

November 14, 2025: Clone of the Trident platform code, forked for use on "Big League"

This is a dummy change.


/stdcard/FIRST/matchlogs/
    Match-0-AutoBlueFar.txt


To download logs from the robot:
  - connect adb
  - adb logcat > logs.txt


League Meet notes:

- first shot out of turret is "too fast" -- maybe target hasn't been set yet, so it thinks it's good to go?
- obelisk detect seems "decent" but sorting is inconsistent?
- RPM/hood for far zone off
  (seems like the "further away" hood angle just too aggressive?)
- going into other zone on far-side pathing
- several effeciencies available for auto

- "-6 on red" seemed to be a good tweak, during Wed adrian practice

- in both post-auto teleop and straight "blue" teleop the coordinates are weird.
  - we get about the right full-field distance from reset
  - but it changes by ~12 inches when rotated ("correct" at 0 and 90, "~12 inches shorter" at 180 and 270)


Pedro Pathing notes:

- "hold end" is to allow follower.update() to "continue to correct" after the path is "done"
- PathChain vs Path seems to be inconsistent wrt setBrakingStrength() (only had success with "Path")
- setGlobalDeceleration(X) is same as .setBrakingStrength(X)
- defaults for all tunable values: https://github.com/Pedro-Pathing/PedroPathing/blob/main/core/src/main/java/com/pedropathing/follower/FollowerConstants.java
- 

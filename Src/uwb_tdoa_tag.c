/*
 * TDOA tag algorithm
 *  Anchors broadcast their timestamps one by one.
 *  Based on anchors' timestamps and TOA of the msg, the tag calculate the distances among
 *  anchors and anchors, the TDOA between tag and anchors, and finally the coordinate of
 *  the tag, whitch is the intersection of at least 3 hyperboles.
 *  Tags don't send messages, so that it occupies no bandwidth, and the number of tags is
 *  not limitted.
 */
#include <uwb.h>
#include <stdio.h>
#include <string.h>

/*
 * Timestamps of the anchors
 * Mark down the TOA of the messages, and get the timestamps from anchors.
 * First we calculate the TOF between anchors, thus the distance between anchors.
 * Then we calculate TDOA one by one between tag and anchor, thus the difference of 
 * distance between tag and anchor, and finally produce the tag's location.
 */

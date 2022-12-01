#!/bin/bash

rosrun image_view stereo_view stereo:=stereo image:=image_rect _approximate_sync:=True

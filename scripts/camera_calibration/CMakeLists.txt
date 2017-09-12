
if(OpenCV_FOUND)

	add_executable(
		camera_calibration

		camera_calibration.cpp
	)

	target_link_libraries(camera_calibration ${OpenCV_LIBS})

endif()

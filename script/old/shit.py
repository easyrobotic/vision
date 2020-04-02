def hola():
	"""
	lab = cv2.cvtColor(result, cv2.COLOR_BGR2LAB)
	gray = cv2.cvtColor(lab, cv2.COLOR_BGR2GRAY)
	thresh = cv2.threshold(result, 0, 50, cv2.THRESH_BINARY)[1]
	cv2.imshow("lab_result", lab)
	cv2.imshow("gray_result", gray)
	cv2.imshow("tresh_result", thresh)
	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	sd = ShapeDetector()
	"""
		"""for c in cnts:
			# compute the center of the contour
			M = cv2.moments(c)
			cX = int((M["m10"] / M["m00"]) * ratio)
			cY = int((M["m01"] / M["m00"]) * ratio)
			# detect the shape of the contour and label the color
			shape = sd.detect(c)
			print(shape)
			#color = cl.label(lab, c)
			# multiply the contour (x, y)-coordinates by the resize ratio,
			# then draw the contours and the name of the shape and labeled
			# color on the image
			#c = c.astype("float")
			#c *= ratio
			#c = c.astype("int")
			#text = "{} {}".format(color, shape)
			cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
			#cv2.putText(image, text, (cX, cY),
			#	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			# show the output image
			cv2.imshow("Image_final", image)
			cv2.waitKey(0)"""
		#b, g, r = cv2.split(result)
		#cv2.imshow("Image_blau", image_)
		#g = OCR.clahe(g, 5, (5, 5))
		#inverse = cv2.bitwise_not(g)
		#blue,green,red = cv2.split(image_);
		#cv2.imshow("Image_blau", image_)
		#cv2.imshow(aa"Image_verda", green)
		#cv2.imshow("Image_roja", red)
		#cv2.imshow("Image_blau", image_[0])
		#cv2.imshow("Image_verda", image_[1])
		#cv2.imshow("Image_roja",image_[2])
		"""
		resized = imutils.resize(image, width=700)
		ratio = image.shape[0] / float(resized.shape[0])

		# blur the resized image slightly, then convert it to both
		# grayscale and the L*a*b* color spaces
		blurred = cv2.GaussianBlur(resized, (5, 5), 0)
		#cv2.imshow("Image_blurred", resized)
		gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
		#cv2.imshow("Image_gray", gray)
		lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
		#cv2.imshow("Image_lab", lab)
		#thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)[1]
		thresh = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)[1]
		#cv2.imshow("tresh", thresh)
		# find contours in the thresholded image
		cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		# initialize the shape detector and color labeler
		sd = ShapeDetector()
		cl = ColorLabeler()
		# loop over the contours
		for c in cnts:
			# compute the center of the contour
			M = cv2.moments(c)
			cX = int((M["m10"] / M["m00"]) * ratio)
			cY = int((M["m01"] / M["m00"]) * ratio)
			# detect the shape of the contour and label the color
			shape = sd.detect(c)
			color = cl.label(lab, c)
			# multiply the contour (x, y)-coordinates by the resize ratio,
			# then draw the contours and the name of the shape and labeled
			# color on the image
			c = c.astype("float")
			c *= ratio
			c = c.astype("int")
			text = "{} {}".format(color, shape)
			cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
			cv2.putText(image, text, (cX, cY),
				cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			# show the output image
			cv2.imshow("Image_final", image)
			cv2.waitKey(0)
		"""

        """
        def DetectColor(image_):
        	#ap = argparse.ArgumentParser()
        	#ap.add_argument("-i", "--image", required=True,
        	#	help="path to the input image")
        	#args = vars(ap.parse_args())

        	# load the image and resize it to a smaller factor so that
        	# the shapes can be approximated better
        	#image = cv2.imread(image_)
        	#resized = imutils.resize(image_, width=300)
        	cv2.imshow("Image_blau", image_[0])
        	cv2.imshow("Image_verda", image_[1])
        	cv2.imshow("Image_roja",image_[2])

        	resized = imutils.resize(image_, width=700)
        	ratio = image_.shape[0] / float(resized.shape[0])

        	# blur the resized image slightly, then convert it to both
        	# grayscale and the L*a*b* color spaces
        	blurred = cv2.GaussianBlur(resized, (5, 5), 0)
        	cv2.imshow("Image_blurred", resized)
        	gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        	cv2.imshow("Image_gray", gray)
        	lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
        	cv2.imshow("Image_lab", lab)
        	#thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)[1]
        	thresh = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)[1]
        	cv2.imshow("tresh", thresh)
        	# find contours in the thresholded image
        	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        	cnts = imutils.grab_contours(cnts)
        	# initialize the shape detector and color labeler
        	sd = ShapeDetector()
        	cl = ColorLabeler()
        	# loop over the contours
        	for c in cnts:
        		# compute the center of the contour
        		M = cv2.moments(c)
        		cX = int((M["m10"] / M["m00"]) * ratio)
        		cY = int((M["m01"] / M["m00"]) * ratio)
        		# detect the shape of the contour and label the color
        		shape = sd.detect(c)
        		color = cl.label(lab, c)
        		# multiply the contour (x, y)-coordinates by the resize ratio,
        		# then draw the contours and the name of the shape and labeled
        		# color on the image
        		c = c.astype("float")
        		c *= ratio
        		c = c.astype("int")
        		text = "{} {}".format(color, shape)
        		cv2.drawContours(image_, [c], -1, (0, 255, 0), 2)
        		cv2.putText(image_, text, (cX, cY),
        			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        		# show the output image
        		cv2.imshow("Image_final", image_)
        		cv2.waitKey(0)
        """

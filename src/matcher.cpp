#include "matcher.h"

template<typename T, typename C>
vector<T> pq_to_vector(priority_queue<T, vector<T>, C> pq) {
	vector<T> vec;
	while (!pq.empty()) {
		T elem = pq.top();
		vec.push_back(elem);
		pq.pop();
	}
	return vec;
}

// Template match template on base image over range of scales and rotations
vector<TemplateMatch> match(const cv::Mat &img, NeedleTemplate templ) {
	// Record matching time
	auto t = (double)cv::getTickCount();

	// Note: this "params" setup isn't necessary for final implementation.
	// This is only used to "cheat" the system by allowing for known poses
	// to have their scale and rotation bounds preloaded from the
	// PfcInitConstants.hpp file. In a final implementation, the
	// rotation and scaling ranges and increments would just be
	// referenced directly from the PfcInitConstants.hpp file, or
	// some other configuration file
	double min_y = templ.params.min_yaw, max_y = templ.params.max_yaw;
	double y_inc = templ.params.yaw_inc;
	double min_p = templ.params.min_pitch, max_p = templ.params.max_pitch;
	double p_inc = templ.params.pitch_inc;
	double min_r = templ.params.min_roll, max_r = templ.params.max_roll;
	double r_inc = templ.params.roll_inc;
	double min_z = templ.params.min_z, max_z = templ.params.max_z;
	double z_inc = templ.params.z_inc;

	// Track matches
	vector<TemplateMatch> matches;

	// For all z
	double z = min_z;
	while (z < max_z) {
		auto tz = (double)cv::getTickCount();
		// Fpr all yaw (y values)
		double y = min_y;
		while (y < max_y) {
			auto ta = (double)cv::getTickCount();
			//For all pitch (p values)
			double p = min_p;
			while (p < max_p) {
				auto tb = (double)cv::getTickCount();
				// For all roll (r values)
				double r = min_r;
				while (r < max_r) {
					// Generate Template
					templ.GenerateTemplate(z, y, p, r);
					//Match rotated template to image
					TemplateMatch new_match = getMatch(img, templ.image);

					// Store other match details
					new_match.z = z;
					new_match.yaw = constrainAngle(y, false);
					new_match.pitch = constrainAngle(p, false);
					new_match.roll = constrainAngle(r, false);
					// Offset the origin to sit in the correct location of the final image
					new_match.origin = templ.origin + cv::Point2d(new_match.rect.x, new_match.rect.y);
					matches.push_back(new_match);

					r += r_inc;
				}
				// tb = ((double)cv::getTickCount() - tb) / cv::getTickFrequency();
				// cout << "p step time: " << tb << " s" << endl;
				p += p_inc;
			}
			// ta = ((double)cv::getTickCount() - ta) / cv::getTickFrequency();
			// cout << "y step time: " << ta << " s" << endl;
			y += y_inc;
		}
		// tz = ((double)cv::getTickCount() - tz) / cv::getTickFrequency();
		// cout << "Z step time: " << tz << " s" << endl;
		z += z_inc;
	}

	// t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	// cout << "thread time: " << t << " s" << endl;
	return matches;
}

// Equivalent to match but splitting scale range amongst # of parallel threads
vector<TemplateMatch> matchThreaded(const cv::Mat &img, NeedleTemplate templ) {
	// Init thread matches and thread return values (futures)
	std::vector<std::future<vector<TemplateMatch>>> futures;

	// Get range
	double max_val = templ.params.max_pitch;
	double min_val = templ.params.min_pitch;
	double inc_val = templ.params.pitch_inc;

	// Calc num threads and scale increment between sequential threads
	int num_threads = get_nprocs();

	double thread_inc = ((max_val - min_val)/(double)num_threads);
	// If increment > thread inc, threads do nothing
	if (thread_inc < inc_val) {
		thread_inc = inc_val - 0.01;
		num_threads = ceil((max_val - min_val)/inc_val);
	}

	// Loop to launch threads
	for (int tid = 0; tid < num_threads; tid++) {
		cout << "launching thread: " << tid << endl;

		// Compute thread range
		templ.params.min_pitch = (min_val + tid*thread_inc);
		cout << templ.params.min_pitch << endl;

		// Account for edge case max_pitch / num_threads == pitch_inc, threads do nothing
		if (max_val/(double)num_threads==inc_val)
			templ.params.max_pitch = min_val + (tid + 1)*thread_inc + 0.0001;
		else
			templ.params.max_pitch = min_val + (tid + 1)*thread_inc;

		cout << templ.params.max_pitch << endl;
		// Round to correct decimal place (this rounds to 2 places, only works
		// for increments of 2 decimal place as well)
		templ.params.min_pitch = round(templ.params.min_pitch*100)/100;
		templ.params.max_pitch = round(templ.params.max_pitch*100)/100;

		// Except first, shift range to avoid overlap
		if (tid!=0) {
			templ.params.min_pitch += inc_val;
		}

		// Launch thread and collect future
		futures.push_back(std::async(launch::async, match, img, templ));
		// cout << "launched: " << tid << " range: " << templ.params.min_yaw << ", " << templ.params.max_yaw << endl;
	}


	// Extract matches from futures
	std::vector<TemplateMatch> all_matches;
	for (auto &fut : futures) {
		vector<TemplateMatch> thread_matches = fut.get();
		all_matches.insert(all_matches.end(), thread_matches.begin(), thread_matches.end());
	}

	return collect_top_matches(all_matches, templ.params.num_cand_pts);
}

vector<TemplateMatch> collect_top_matches(vector<TemplateMatch> matches, int num_cand_pts) {
	priority_queue<TemplateMatch, vector<TemplateMatch>, TemplateMatchComparator> best_matches;

	for(const auto& m : matches) {
		// If queue not full
		if (best_matches.size() < num_cand_pts) {
			// Insert match
			best_matches.push(m);
		}
		// If queue full and score is better than min score
		else if (m.score > best_matches.top().score) {
			// Delete the smallest score match
			best_matches.pop();
			// Add the new better match
			best_matches.push(m);
		}
//		cout << m.score << endl;
//		cout << "worst score: " << best_matches.top().score << endl;
		// Otherwise reject the match since it isn't in the top n scores
	}
	return pq_to_vector(best_matches);
}

// Run template match and store results if this match is better than bestMatch
TemplateMatch getMatch(const cv::Mat &img, const cv::Mat &templ) {
	// Create the result matrix
	cv::Mat result;

	//Match using TM_CCOEFF_NORMED
	cv::matchTemplate(img, templ, result, cv::TM_CCOEFF_NORMED);

	/// Localizing the best match with minMaxLoc
	double minVal;
	double maxVal;
	cv::Point minLoc;
	cv::Point maxLoc;
	cv::Point matchLoc;
	cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

	// Create template match
	cv::Rect2i rect(maxLoc.x, maxLoc.y, templ.cols, templ.rows);

	TemplateMatch match =
		{
			maxVal,
			rect,
			result,
			templ
		};

	return match;
}



/**
 * These methods were prototyping for a different scoring method
 */
//double match_rot_dist(const TemplateMatch *m1, const TemplateMatch *m2) {
//	Eigen::Vector4d orientation1(m1->roll, m1->pitch, m1->yaw, m1->z);
//	Eigen::Vector4d orientation2(m2->roll, m2->pitch, m2->yaw, m2->z);
//	cout << orientation1 << endl;
//	cout << orientation2 << endl;
//	return (orientation1 - orientation2).norm();
//}
//
//bool match_too_close(TemplateMatch *new_match, const vector<TemplateMatch> *matches) {
//	for (int i = 0; i < matches->size(); i++) {
//		TemplateMatch m = matches->at(i);
//		double dist = match_rot_dist(new_match, &m);
//		if (dist < pfc::match_dist_thresh) {
//			return true;
//		}
//	}
//	return false;
//}
//
//void set_min_score(vector<TemplateMatch> *matches, double *min_score, int *min_idx) {
//	for (int i = 0; i < matches->size(); i++) {
//		auto m = matches->at(i);
//		if (m.score < *min_score) {
//			min_score = &m.score;
//			min_idx = &i;
//		}
//	}
//}
//
//vector<TemplateMatch> select_top_matches(vector<TemplateMatch> *matches, int num_cand_pts) {
//	vector<TemplateMatch> best_matches;
//	best_matches.push_back(matches->at(0));
//	// Track min score and index
//	auto min_score = matches->at(0).score;
//	int min_idx = 0;
//
//	// Loop over all matches
//	for (TemplateMatch match : *matches) {
//		double score = match.score; // matching score
//
//		// If new score is better than min score
//		if (score > min_score) {
//			// Save the min
//			TemplateMatch prev_min = best_matches.at(min_idx);
//
//			// If reached desired num of pts, remove the min before distance comparison
//			// Since the new match has a better matching score, even if it's too close to the min then
//			// it should replace the min. So we compare distance on list without the current min
//			if (best_matches.size() >= num_cand_pts) {
//				best_matches.erase(best_matches.begin() + min_idx - 1);
//			}
//
//			// If point not too close to any others already in list
//			if (!match_too_close(&match, &best_matches)) {
//				// Add it, find new min
//				best_matches.push_back(match);
//				set_min_score(&best_matches, &min_score, &min_idx);
//			} else {
//				// If too close to any pts, add the old min (that wasn't too close) back in
//				if(best_matches.size() != 1)  // avoid duplicating first entry
//					best_matches.push_back(prev_min);
//				min_score = prev_min.score;
//				min_idx = best_matches.size() - 1;
//			}
//		}
//	}
//}
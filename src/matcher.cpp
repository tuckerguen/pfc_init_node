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
	for (double z = min_z; z < max_z; z += z_inc) {
		auto tz = (double)cv::getTickCount();
		// For all yaw (y values)
		for (double y = min_y; y < max_y; y += y_inc) {
			auto ta = (double)cv::getTickCount();
			//For all pitch (p values)
			while (double p = min_p; p < max_p; p += p_inc) {
				auto tb = (double)cv::getTickCount();
				// For all roll (r values)
				while (double r = min_r; r < max_r; r += r_inx) {
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
				}
			}
		}
	}

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
		templ.params.max_pitch = min_val + (tid + 1)*thread_inc;
		if (max_val/(double)num_threads==inc_val)
			templ.params.max_pitch += 0.0001;

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
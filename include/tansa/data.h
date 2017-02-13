#ifndef TANSA_DATA_H
#define TANSA_DATA_H

#include "json.hpp"

#include <Eigen/Dense>
#include <memory>
#include <string>

using json = nlohmann::json;

using namespace Eigen;

namespace tansa {

/**
 * Common class for reading data from files
 */
class DataObject {
public:

	static DataObject LoadFile(std::string filename);


	DataObject(const json &j) {
		this->data = j;
	}

	template<typename T>
	DataObject operator[](T key) const {
		return DataObject(data[key]);
	}




	template<int N, int M>
	Matrix<double, N, M> matrix() const {
		Matrix<double, N, M> m;
		for(unsigned i = 0; i < N; i++) {
			for(unsigned j = 0; j < M; j++) {
				if(N == 0 || M == 0) { // Case it is a vector
					m(i, j) = data[i + j];
				}
				else {
					m(i, j) = data[i][j];
				}
			}
		}

		return m;
	}

	// Allow casting to all types supported by the json library
	template<typename T>
	operator T() const {
		return data;
	}


	template<typename T>
	DataObject &operator=(T val) {
		data = val;
		return *this;
	}


private:
	json data;

};


}



#endif

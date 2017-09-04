


// Ground Truth points
unsigned N = 10;
vector<Vector3d> pts;
for(unsigned i = 0; i < N; i++) {
	pts.push_back(Vector3d(5*RANDFLOAT, 5*RANDFLOAT, 5*RANDFLOAT));
}


// Estimates of point locations
VectorXd x(N*3);
for(unsigned i = 0; i < N*3; i++) {
	x(i) = 5*RANDFLOAT;
}

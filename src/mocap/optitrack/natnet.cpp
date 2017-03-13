#include "natnet.h"

namespace tansa {
namespace optitrack {

template<typename T>
inline void buffer_read(const char *&buf, T &out) {
	unsigned size = sizeof(T);
	memcpy(&out, buf, size);
	buf += size;
}



const char *NatNetRigidBody::Parse(const char *ptr, const NatNetVersion v, NatNetRigidBody *rb) {
	int major = v[0], minor = v[1];

	int n;

	// rigid body pos/ori
	buffer_read(ptr, rb->id);
	buffer_read(ptr, rb->x);
	buffer_read(ptr, rb->y);
	buffer_read(ptr, rb->z);
	buffer_read(ptr, rb->qx);
	buffer_read(ptr, rb->qy);
	buffer_read(ptr, rb->qz);
	buffer_read(ptr, rb->qw);

	buffer_read(ptr, n); rb->markerPositions.resize(n);
	int nb = n * sizeof(NatNetPlainMarker);
	memcpy(&rb->markerPositions[0], ptr, nb); ptr += nb;

	if(major >= 2) {
		nb = n * sizeof(int32_t);
		rb->markerIds.resize(n);
		memcpy(&rb->markerIds[0], ptr, nb); ptr += nb;

		nb = n * sizeof(float);
		rb->markerSizes.resize(n);
		memcpy(&rb->markerSizes[0], ptr, nb); ptr += nb;
	}

	if(major >= 2) {
		buffer_read(ptr, rb->meanError);
	}

	// 2.6 and later
	if(((major == 2) && (minor >= 6)) || (major > 2) || (major == 0)) {
		buffer_read(ptr, rb->params);
	}

	return ptr;
}


const char *NatNetFrame::Parse(const char *ptr, const NatNetVersion v, NatNetFrame *frame) {
	int major = v[0], minor = v[1];

	uint32_t n; // A temporary location to put the array sizes
	unsigned i, j, nb;

	memset(frame, 0, sizeof(NatNetFrame));

	buffer_read(ptr, frame->number);

	buffer_read(ptr, n);
	frame->markerSets.resize(n);
	for(i = 0; i < frame->markerSets.size(); i++) {
		NatNetMarkerSet &set = frame->markerSets[i];

		strcpy(set.name, ptr);
		ptr += strlen(set.name) + 1;

		buffer_read(ptr, n);
		set.markers.resize(n);

		nb = n * sizeof(NatNetPlainMarker);
		memcpy(&set.markers[0], ptr, nb); ptr += nb;
	}


	// unlabeled / unidentified markers
	buffer_read(ptr, n);
	frame->otherMarkers.resize(n);
	nb = n * sizeof(NatNetPlainMarker);
	memcpy(&frame->otherMarkers[0], ptr, nb); ptr += nb;


	// rigid bodies
	buffer_read(ptr, n); frame->rigidBodies.resize(n);
	for(i = 0; i < n; i++) {
		ptr = NatNetRigidBody::Parse(ptr, v, &frame->rigidBodies[i]);
	}


	// skeletons (version 2.1 and later)
	if(((major == 2) && (minor > 0)) || (major > 2)) {
		buffer_read(ptr, n); frame->skeletons.resize(n);
		for(i = 0; i < frame->skeletons.size(); i++) {
			NatNetSkeleton &skel = frame->skeletons[i];

			buffer_read(ptr, skel.id);

			buffer_read(ptr, n); skel.rigidBodies.resize(n);
			for(j = 0; j < skel.rigidBodies.size(); j++) {
				ptr = NatNetRigidBody::Parse(ptr, v, &skel.rigidBodies[j]);
			}
		}
	}


	// labeled markers (version 2.3 and later)
	if(((major == 2) && (minor >= 3)) || (major > 2)) {
		buffer_read(ptr, n); frame->labeledMarkers.resize(n);

		for(i = 0; i < n; i++) {
			NatNetMarker &m = frame->labeledMarkers[i];

			// id, x, y, z, size
			memcpy(&m, ptr, 20); ptr += 20;

			// 2.6 and later
			if(((major == 2) && (minor >= 6)) || (major > 2) || (major == 0)) {
				// marker params
				buffer_read(ptr, m.params);
			}
		}
	}

	// Force Plate data (version 2.9 and later)
	if(((major == 2) && (minor >= 9)) || (major > 2)) {

		buffer_read(ptr, n); frame->forcePlates.resize(n);
		for(i = 0; i < frame->forcePlates.size(); i++){
			NatNetForcePlate &fp = frame->forcePlates[i];

			buffer_read(ptr, fp.id);

			buffer_read(ptr, n); fp.channels.resize(n);
			for(j = 0; j < fp.channels.size(); j++) {
				NatNetAnalogChannel &chan = fp.channels[j];

				buffer_read(ptr, n); chan.framesValue.resize(n);
				for(int k = 0; k < n; k++) {
					buffer_read(ptr, chan.framesValue[k]);
				}
			}
		}
	}

	// latency
	buffer_read(ptr, frame->latency);

	// timecode
	buffer_read(ptr, frame->timecode.code);
	buffer_read(ptr, frame->timecode.subframe);

	// timestamp
	// 2.7 and later - increased from single to double precision
	if(((major == 2) && (minor >= 7)) || (major > 2)) {
		buffer_read(ptr, frame->timestamp);
	}
	else {
		float temp = 0.0f;
		buffer_read(ptr, temp);
		frame->timestamp = (double) temp;
	}

	buffer_read(ptr, frame->params);


	// TODO: Figure out what this marker is and validate it
	// end of data tag
	int32_t eod = 0; memcpy(&eod, ptr, 4); ptr += 4;

	return ptr;
}

const char *NatNetRigidBody::Description::Parse(const char *ptr, const NatNetVersion v, NatNetRigidBody::Description *rb) {
	int major = v[0];

	if(major >= 2) {
		strcpy(rb->name, ptr);
		ptr += strlen(ptr) + 1;
	}

	buffer_read(ptr, rb->id);
	buffer_read(ptr, rb->parent_id);
	buffer_read(ptr, rb->offset_x);
	buffer_read(ptr, rb->offset_y);
	buffer_read(ptr, rb->offset_z);

	return ptr;
}


void NatNetDescriptions::Parse(const char *ptr, const NatNetVersion v, NatNetDescriptions *descs) {

	uint32_t n;


	buffer_read(ptr, n);
	descs->arr.resize(n);

	for(int i = 0; i < descs->arr.size(); i++){
		NatNetDescription &desc = descs->arr[i];

		buffer_read(ptr, desc.type);

		if(desc.type == NatNetDescription::MarkerSet) {
			NatNetMarkerSet::Description *ms = new NatNetMarkerSet::Description();
			desc.data = ms;

			strcpy(ms->name, ptr);
			ptr += strlen(ms->name) + 1;

			buffer_read(ptr, n); ms->markerNames.resize(n);
			for(int j = 0; j < n; j++){
				char buf[256];
				strcpy(buf, ptr);
				ptr += strlen(buf) + 1;
				ms->markerNames[j] = std::string(buf);
			}
		}
		else if(desc.type == NatNetDescription::RigidBody) {
			NatNetRigidBody::Description *rb = new NatNetRigidBody::Description();
			desc.data = rb;

			ptr = NatNetRigidBody::Description::Parse(ptr, v, rb);
		}
		else if(desc.type == NatNetDescription::Skeleton) {
			NatNetSkeleton::Description *skel = new NatNetSkeleton::Description();
			desc.data = skel;

			strcpy(skel->name, ptr);
			ptr += strlen(ptr) + 1;

			buffer_read(ptr, skel->id);

			buffer_read(ptr, n); skel->rigidBodies.resize(n);
			for(int j = 0; j < n; j++) {
				NatNetRigidBody::Description::Parse(ptr, v, &skel->rigidBodies[j]);
			}
		}

	}
}



void NatNetTimecode::decode(int *hour, int *minute, int *second, int *frame, int *subframe) {
	*hour = (code >> 24) & 255;
	*minute = (code >> 16) & 255;
	*second = (code >> 8) & 255;
	*frame = code & 255;
	*subframe = this->subframe;
}

std::string NatNetTimecode::to_string() {

	int hour, minute, second, frame, subframe;
	this->decode(&hour, &minute, &second, &frame, &subframe);

	char buf[128];
	snprintf(buf, sizeof(buf), "%02d:%02d:%02d:%02d.%d", hour, minute, second, frame, subframe);

	return std::string(buf);
}


}
}

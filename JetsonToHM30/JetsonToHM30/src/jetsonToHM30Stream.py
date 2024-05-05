#sean broderick
import cv2
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Initialize GStreamer
Gst.init(None)

# Define the pipeline for encoding and streaming with H.265
# Replace 'host=192.168.1.100 port=5000' with your HM30's IP and the appropriate port
streaming_pipeline = (
    'appsrc name=source is-live=true block=true format=GST_FORMAT_TIME ! '
    'videoconvert ! '
    'x265enc tune=zerolatency bitrate=5000 speed-preset=ultrafast ! ' #make sure HM30 allows H.265 encoding and playing around for optimal bitrate value
    'rtph265pay config-interval=1 pt=96 ! '
    'udpsink host=192.168.1.100 port=5000'
)
#adjust streaming settings form optimal performance. 

# Create the GStreamer pipeline
pipeline = Gst.parse_launch(streaming_pipeline)
appsrc = pipeline.get_by_name('source')
pipeline.set_state(Gst.State.PLAYING)

def push_frame_to_pipeline(frame):
    """Converts a frame to GstBuffer and pushes it to the pipeline"""
    # Convert the frame to GstBuffer
    data = frame.tostring()
    buffer = Gst.Buffer.new_allocate(None, len(data), None)
    buffer.fill(0, data)
    
    # Set timestamp and duration
    pts = Gst.CLOCK_TIME_NONE  # Use current time
    buffer.pts = buffer.dts = pts
    buffer.duration = Gst.CLOCK_TIME_NONE
    
    # Push the buffer into the pipeline
    result = appsrc.emit('push-buffer', buffer)
    if result != Gst.FlowReturn.OK:
        print('Error pushing buffer to pipeline')



# Use this function to push each processed frame to the pipeline
 #(might need to convert to C++ in order for this to work and call in inference script)
#for frame in frames:
 #   push_frame_to_pipeline(frame)

#opencv video streamer and send each frome to here. opencv example video code
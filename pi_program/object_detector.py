""" SICKSense Agripollinate Object Detector Module Version 1.0
    Created by:    Josiah Faircloth 
    date:    11/30/2025

    Real-time object detection and tracking from SICK TiM561 LiDAR scan data.
    Adapted from LiDAR Detector Version 0.3 for streaming data integration.
"""

import numpy as np
from sklearn.cluster import DBSCAN


class ObjectDetector:
    """
    Detects and tracks object clusters from LiDAR scan data in real-time.
    Triggers callbacks when new objects are detected.
    """
    
    def __init__(self, 
                 eps=0.25, 
                 min_samples=3,
                 new_obj_dist=0.3,
                 object_memory_time=1.0,
                 min_detections=3,
                 trigger_cooldown=2.0,
                 established_threshold=2.0):
        """
        Initialize object detector with configuration parameters.
        
        Args:
            eps: DBSCAN cluster distance threshold (meters)
            min_samples: Minimum points per cluster
            new_obj_dist: Distance threshold for new object classification (meters)
            object_memory_time: Time to remember objects after last seen (seconds)
            min_detections: Number of consecutive scans before triggering
            trigger_cooldown: Minimum time between triggers (seconds)
            established_threshold: Time to be considered "established" (seconds)
        """
        self.eps = eps
        self.min_samples = min_samples
        self.new_obj_dist = new_obj_dist
        self.object_memory_time = object_memory_time
        self.min_detections = min_detections
        self.trigger_cooldown = trigger_cooldown
        self.established_threshold = established_threshold
        
        # Initialize tracker
        self.tracker = ObjectTracker(
            memory_time=object_memory_time,
            established_threshold=established_threshold,
            distance_threshold=new_obj_dist,
            min_detections=min_detections
        )
        
        # Tracking state
        self.last_trigger_time = None
        self.object_count = 0
        self.on_trigger_callback = None
    
    def set_trigger_callback(self, callback):
        """
        Set callback function to be called when new object is detected.
        Callback signature: callback(centroid, timestamp, object_count)
        """
        self.on_trigger_callback = callback
    
    def polar_to_cartesian(self, ranges):
        """Convert polar ranges to (x, y) coordinates"""
        angles = np.linspace(-45, 225, len(ranges))
        x = ranges * np.cos(np.deg2rad(angles))
        y = ranges * np.sin(np.deg2rad(angles))
        return np.column_stack((x, y))  # (N, 2) array of (x, y) points

    
    def detect_objects(self, points):
        """Cluster points using DBSCAN and return cluster centroids"""
        points = points[~np.isnan(points).any(axis=1)] # Remove NaN points
        if len(points) < self.min_samples: # Exit if not enough points to form clusters
            return []
        
        db = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points)
        labels = db.labels_ # Cluster labels for each point
        centroids = []
        
        for lbl in np.unique(labels):
            if lbl == -1:  # noise
                continue
            cluster = points[labels == lbl] 
            centroids.append(np.mean(cluster, axis=0)) # Compute centroid of cluster
        
        return np.array(centroids) # (M, 2) array of centroids

    def process_scan(self, scan_data):
        """
        Process a single LiDAR scan and detect new objects.
        
        Args:
            scan_data: Dict with 'timestamp', 'num_points', 'ranges'
        
        Returns:
            Dict with 'new_objects', 'tracking_info', 'triggered'
        """
        # Extract and preprocess ranges
        ranges = np.array(scan_data["ranges"])
        ranges[(ranges < 0.005) | (ranges > 10)] = np.nan
        
        # Convert to cartesian and detect objects
        points = self.polar_to_cartesian(ranges)
        centroids = self.detect_objects(points)
        
        # Update tracker
        new_objects, tracking_info = self.tracker.update(centroids, scan_data["timestamp"])
        
        # Check for triggers
        triggered = False
        if len(new_objects) > 0:
            # Check cooldown
            current_time = scan_data["timestamp"]
            if self.last_trigger_time is None or \
               current_time - self.last_trigger_time >= self.trigger_cooldown:
                
                triggered = True
                self.last_trigger_time = current_time
                self.object_count += 1
                
                # Call trigger callback if set
                if self.on_trigger_callback:
                    for centroid in new_objects:
                        self.on_trigger_callback(centroid, current_time, self.object_count)
        
        return {
            'new_objects': new_objects,
            'tracking_info': tracking_info,
            'triggered': triggered,
            'points': points,
            'centroids': centroids
        }
    
    def get_statistics(self):
        """Return detection statistics"""
        return {
            'total_objects_detected': self.object_count,
            'last_trigger_time': self.last_trigger_time,
            'active_objects': len(self.tracker.tracking_objects),
            'candidate_objects': len(self.tracker.candidate_objects)
        }


class ObjectTracker:
    """Tracks known objects over time with memory persistence and confirmation"""
    
    def __init__(self, memory_time=2.0, established_threshold=2.0, 
                 distance_threshold=0.4, min_detections=3):
        self.candidate_objects = []  # List of (centroid, first_seen_time, detection_count)
        self.tracking_objects = {} # {object_id: {'position': np.array, 'init_pos': np.array,'first_seen': float,'last_seen': float, 'status': str}}
        self.next_object_id = 0
        self.established_threshold = established_threshold
        self.memory_time = memory_time
        self.distance_threshold = distance_threshold
        self.min_detections = min_detections
        self.first_scan = True
    
    def update(self, current_centroids, current_time):
        """Update tracker with new centroids, return list of NEW objects"""
        # On first scan, add all objects without triggering
        if self.first_scan:
            for centroid in current_centroids:
                # Add to tracking objects as "established"
                obj_id = self.next_object_id
                self.next_object_id += 1
                self.tracking_objects[obj_id] = {
                    'position': centroid.copy(),
                    'init_pos': centroid.copy(),
                    'first_seen': current_time,
                    'last_seen': current_time,
                    'status': 'established'
                }
            self.first_scan = False
            return np.array([]).reshape(0, 2), self.tracking_objects.copy()
        
        # Clean up old tracking objects
        ids_to_remove = []
        for obj_id, obj_data in self.tracking_objects.items():
            if current_time - obj_data['last_seen'] > self.memory_time:
                ids_to_remove.append(obj_id)
        for obj_id in ids_to_remove:
            del self.tracking_objects[obj_id]
        
        # Clean up old candidates
        self.candidate_objects = [(pos, first_t, count) for pos, first_t, count 
                                  in self.candidate_objects 
                                  if current_time - first_t < self.memory_time]
        
        new_objects = []
        matched_candidates = set()
        
        for centroid in current_centroids:
            matched = False
            
            # Check if matches an existing tracking object
            for obj_id, obj_data in self.tracking_objects.items():
                dist = np.linalg.norm(centroid - obj_data['position'])
                if dist < self.distance_threshold:
                    # Update tracking object
                    obj_data['position'] = centroid.copy()
                    obj_data['last_seen'] = current_time
                    
                    # Update status based on time tracked
                    if obj_data['status'] == 'established':
                        if np.linalg.norm(centroid - obj_data['init_pos']) > self.distance_threshold:
                            new_objects.append(centroid)
                            obj_data['status'] = 'tracking'
                    else:
                        time_tracked = current_time - obj_data['first_seen']
                        if time_tracked >= self.established_threshold:
                            obj_data['status'] = 'established'
                        else:
                            obj_data['status'] = 'tracking'
                    
                    matched = True
                    break
            
            if matched:
                continue
            
            # Check if matches a candidate object
            for i, (cand_pos, first_t, count) in enumerate(self.candidate_objects):
                dist = np.linalg.norm(centroid - cand_pos)
                if dist < self.distance_threshold:
                    new_count = count + 1
                    if new_count >= self.min_detections:
                        new_objects.append(centroid)
                        
                        obj_id = self.next_object_id
                        self.next_object_id += 1
                        self.tracking_objects[obj_id] = {
                            'position': centroid.copy(),
                            'init_pos': centroid.copy(),
                            'first_seen': current_time,
                            'last_seen': current_time,
                            'status': 'new'
                        }
                        matched_candidates.add(i)
                    else:
                        self.candidate_objects[i] = (centroid, first_t, new_count)
                        matched_candidates.add(i)
                    matched = True
                    break
            
            if not matched:
                # New candidate object (first detection)
                self.candidate_objects.append((centroid, current_time, 1))
        
        # Remove matched candidates that were promoted
        self.candidate_objects = [c for i, c in enumerate(self.candidate_objects) 
                                  if i not in matched_candidates or c[2] < self.min_detections]
        
        # Return new objects and tracking status dictionary
        if new_objects:
            return np.array(new_objects).reshape(-1, 2), self.tracking_objects.copy()
        else:
            return np.array([]).reshape(0, 2), self.tracking_objects.copy()

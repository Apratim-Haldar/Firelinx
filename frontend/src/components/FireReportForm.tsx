import React, { useState, useEffect, useCallback, useRef } from 'react';
import { Flame, MapPin, AlertTriangle, Send, Droplet, Thermometer as ThermometerHot, Camera } from 'lucide-react';
import Map, { NavigationControl, ScaleControl, GeolocateControl, FullscreenControl, AttributionControl, Marker, Popup } from 'react-map-gl';
import toast from 'react-hot-toast';
import Modal from 'react-modal';
import Lottie from 'lottie-react';
import fireAnimation from '../assets/fire-animation.json';
import { formatDDMCoordinates, getCurrentTimestamp } from '../utils/coordinates';
import { initializeMQTT, sendFireAlert } from '../utils/mqtt';
import type { FireReport, Coordinates } from '../types';

Modal.setAppElement('#root');

// API Configuration
const API_BASE_URL = 'http://localhost:5000';

const FireMarker = ({ intensity = 1 }: { intensity?: number }) => {
  const baseSize = 48;
  const sizeIncrement = 24;
  const size = baseSize + (intensity - 1) * sizeIncrement;

  return (
    <div style={{
      width: size,
      height: size,
      marginLeft: -size / 2,
      marginTop: -size,
      cursor: 'pointer',
      pointerEvents: 'auto',
      transition: 'all 0.3s ease'
    }}>
      <Lottie
        animationData={fireAnimation}
        loop={true}
        autoplay={true}
        style={{ width: '100%', height: '100%' }}
      />
    </div>
  );
};

const FireReportForm: React.FC = () => {
  const [formData, setFormData] = useState<Partial<FireReport>>({
    fireType: 'A',
    fireIntensity: '1',
    verified: true,
    stnID: 'W/D',
    user: '',
    userID: '',
    ...getCurrentTimestamp()
  });

  const [userLocation, setUserLocation] = useState<Coordinates | null>(null);
  const [mapLoaded, setMapLoaded] = useState(false);
  const [mapInstance, setMapInstance] = useState<mapboxgl.Map | null>(null);
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [messageDetails, setMessageDetails] = useState<string>('');
  const [popupInfo, setPopupInfo] = useState<{ location: Coordinates, isOpen: boolean } | null>(null);
  const geolocateControlRef = useRef<any>(null);
  const hoverTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  useEffect(() => {
    initializeMQTT();
    return () => {
      if (hoverTimeoutRef.current) {
        clearTimeout(hoverTimeoutRef.current);
      }
    };
  }, []);

  const triggerSOS = async () => {
    try {
      const response = await fetch(`${API_BASE_URL}/trigger-sos`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include'
      });

      if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);

      const data = await response.json();

      if (data.status === 'success') {
        toast.success('SOS alert sent successfully!');
      } else {
        throw new Error(data.message || 'Failed to send SOS');
      }
    } catch (error: any) {
      toast.error(`SOS failed: ${error.message}`);
      console.error('SOS error:', error);
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!userLocation) {
      toast.error('Please select a location on the map.');
      return;
    }

    const { latitude, longitude } = formatDDMCoordinates(userLocation.lat, userLocation.lng);

    const payload: FireReport = {
      ...formData as FireReport,
      latitude,
      longitude
    };

    try {
      const message = await toast.promise(
        sendFireAlert(payload),
        {
          loading: 'Sending emergency alert...',
          success: 'Emergency alert sent successfully!',
          error: 'Failed to send emergency alert'
        }
      );

      setMessageDetails(JSON.stringify(message, null, 2));
      setIsModalOpen(true);
      await triggerSOS();
    } catch (error) {
      console.error('Error sending fire alert:', error);
    }
  };

  const handleGeolocate = useCallback((e: any) => {
    const newLocation = {
      lat: e.coords.latitude,
      lng: e.coords.longitude
    };
    setUserLocation(newLocation);
    setPopupInfo({ location: newLocation, isOpen: true });
    
    // Fly to the new location
    if (mapInstance) {
      mapInstance.flyTo({
        center: [e.coords.longitude, e.coords.latitude],
        zoom: 14,
        essential: true
      });
    }
  }, [mapInstance]);

  const handleMapLoad = useCallback((event: mapboxgl.MapboxEvent) => {
    const map = event.target;
    setMapInstance(map);
    setMapLoaded(true);
    map.resize();
  }, []);

  const handleMapClick = (e: any) => {
    const newLocation = {
      lat: e.lngLat.lat,
      lng: e.lngLat.lng
    };
    setUserLocation(newLocation);
    setPopupInfo({ location: newLocation, isOpen: true });
  };

  const handlePopupClose = () => {
    setPopupInfo(null);
  };

  const handleIntensityChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const newIntensity = e.target.value as FireReport['fireIntensity'];
    setFormData(prev => ({
      ...prev,
      fireIntensity: newIntensity
    }));
  };

  const handleRecognizeFace = async () => {
    try {
      const response = await fetch(`${API_BASE_URL}/recognize`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include'
      });

      if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);

      const data = await response.json();

      if (data.status === 'success') {
        toast.success(data.message);
        setFormData(prev => ({
          ...prev,
          user: data.data.name || '',
          userID: data.data.id || ''
        }));
      } else {
        throw new Error(data.message || 'Unknown error');
      }
    } catch (error: any) {
      toast.error(`Recognition failed: ${error.message}`);
      console.error('Recognition error:', error);
    }
  };

  if (!import.meta.env.VITE_MAPBOX_ACCESS_TOKEN) {
    return (
      <div className="bg-red-500/10 border border-red-500/20 rounded-lg p-4 text-red-500">
        Error: Mapbox access token is not configured.
      </div>
    );
  }

  return (
    <div className="bg-[#1A1F2E] rounded-xl shadow-xl overflow-hidden">
      <div className="p-6 border-b border-gray-700 flex justify-between items-center">
        <h2 className="text-2xl font-bold flex items-center gap-2">
          <Flame className="text-red-500" />
          Fire Report System
        </h2>
        <button
          onClick={triggerSOS}
          className="flex items-center gap-2 px-4 py-2 bg-red-500 hover:bg-red-600 text-white rounded-lg transition-colors shadow-md hover:shadow-red-500/20"
        >
          <AlertTriangle className="w-5 h-5" />
          SOS Alert
        </button>
      </div>

      <form onSubmit={handleSubmit} className="p-6 space-y-8">
        <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
          <div className="space-y-6">
            {/* Fire Type + Intensity */}
            <div>
              <label className=" text-sm font-medium text-gray-300 mb-3 flex items-center gap-2">
                <ThermometerHot className="w-4 h-4 text-orange-500" />
                Fire Type
              </label>
              <div className="grid grid-cols-4 gap-2">
                {['A', 'B', 'C', 'D'].map((type) => (
                  <button
                    key={type}
                    type="button"
                    onClick={() => setFormData({ ...formData, fireType: type as FireReport['fireType'] })}
                    className={`p-4 rounded-lg text-center transition-all ${
                      formData.fireType === type
                        ? 'bg-red-500 text-white shadow-lg shadow-red-500/30'
                        : 'bg-gray-700/50 text-gray-300 hover:bg-gray-700'
                    }`}
                  >
                    {type}
                  </button>
                ))}
              </div>
            </div>

            <div>
              <label className="text-sm font-medium text-gray-300 mb-3 flex items-center gap-2">
                <Droplet className="w-4 h-4 text-blue-500" />
                Fire Intensity
              </label>
              <div className="flex justify-between items-center mb-2">
                <span className="text-sm text-gray-300">Intensity Level</span>
                <span className="text-lg font-bold text-white">{formData.fireIntensity}</span>
              </div>
              <input
                type="range"
                min="1"
                max="4"
                step="1"
                value={formData.fireIntensity}
                onChange={handleIntensityChange}
                className="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer"
              />
              <div className="flex justify-between mt-1 px-1">
                {[1, 2, 3, 4].map((value) => (
                  <span key={value} className="text-xs text-gray-400">{value}</span>
                ))}
              </div>
            </div>

            {/* User Inputs */}
            <div className="space-y-3">
              <label className="block text-sm font-medium text-gray-300">Operator Credentials</label>
              <input
                type="text"
                placeholder="Username"
                value={formData.user}
                onChange={(e) => setFormData({ ...formData, user: e.target.value })}
                className="w-full p-3 bg-gray-700/50 rounded-lg text-white placeholder-gray-400"
              />
              <input
                type="text"
                placeholder="User ID"
                value={formData.userID}
                onChange={(e) => setFormData({ ...formData, userID: e.target.value })}
                className="w-full p-3 bg-gray-700/50 rounded-lg text-white placeholder-gray-400"
              />
              <button
                type="button"
                onClick={handleRecognizeFace}
                className="w-full p-3 bg-blue-500 hover:bg-blue-600 rounded-lg text-white transition-colors flex items-center justify-center gap-2"
              >
                <Camera className="w-5 h-5" />
                Recognize Face
              </button>
            </div>
          </div>

          {/* Map Panel */}
          <div className="space-y-6">
            <label className="text-sm font-medium text-gray-300 mb-3 flex items-center gap-2">
              <MapPin className="w-4 h-4 text-emerald-500" />
              Location
            </label>
            <div className="h-[300px] bg-gray-700/50 rounded-lg overflow-hidden relative">
              {!mapLoaded && (
                <div className="absolute inset-0 flex items-center justify-center bg-gray-800/50 z-10">
                  <div className="animate-pulse text-gray-400">Loading map...</div>
                </div>
              )}
              <Map
                mapboxAccessToken={import.meta.env.VITE_MAPBOX_ACCESS_TOKEN}
                initialViewState={{
                  latitude: 22.5767,
                  longitude: 88.2067,
                  zoom: 12,
                  bearing: 0,
                  pitch: 45
                }}
                style={{ width: '100%', height: '100%' }}
                mapStyle="mapbox://styles/mapbox/satellite-streets-v12"
                onLoad={handleMapLoad}
                attributionControl={false}
                antialias={true}
                onClick={handleMapClick}
              >
                <NavigationControl position="top-right" />
                <ScaleControl position="bottom-left" unit="metric" />
                <GeolocateControl
                  ref={geolocateControlRef}
                  position="top-right"
                  trackUserLocation={true}
                  showAccuracyCircle={false}
                  onGeolocate={handleGeolocate}
                />
                <FullscreenControl position="top-right" />
                <AttributionControl compact={true} />

                {userLocation && (
                  <>
                    <Marker
                      longitude={userLocation.lng}
                      latitude={userLocation.lat}
                    >
                      <FireMarker intensity={parseInt(formData.fireIntensity || '1')} />
                    </Marker>

                    {popupInfo?.isOpen && (
                      <Popup
                        longitude={userLocation.lng}
                        latitude={userLocation.lat}
                        anchor="bottom"
                        onClose={handlePopupClose}
                        closeButton={false}
                        closeOnClick={false}
                        className="fire-popup"
                      >
                        <div className="p-2 text-sm min-w-[150px]">
                          <div className="font-bold mb-1">Fire Details</div>
                          <div>Type: {formData.fireType}</div>
                          <div>Integrity: {formData.fireIntensity}</div>
                          <div className="text-gray-500 text-xs mt-1">
                            {formatDDMCoordinates(userLocation.lat, userLocation.lng).latitude},
                            {formatDDMCoordinates(userLocation.lat, userLocation.lng).longitude}
                          </div>
                        </div>
                      </Popup>
                    )}
                  </>
                )}
              </Map>
              <p className="text-xs text-gray-400 mt-2">
                Click anywhere on the map to select a location, or use the GPS button.
              </p>
            </div>
          </div>
        </div>

        <button
          type="submit"
          disabled={!userLocation}
          className={`w-full py-4 rounded-lg transition-all flex items-center justify-center gap-2 shadow-lg ${
            userLocation
              ? 'bg-gradient-to-r from-red-500 to-red-600 text-white hover:from-red-600 hover:to-red-700'
              : 'bg-gray-600 text-gray-400 cursor-not-allowed'
          }`}
        >
          <Send className="w-5 h-5" />
          Submit Emergency Report
        </button>
      </form>

      <Modal
        isOpen={isModalOpen}
        onRequestClose={() => setIsModalOpen(false)}
        className="absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 bg-[#1A1F2E] p-6 rounded-xl shadow-xl max-w-lg w-full"
        overlayClassName="fixed inset-0 bg-black/50"
      >
        <h3 className="text-xl font-bold mb-4 text-white">Message Details</h3>
        <pre className="bg-gray-800/50 p-4 rounded-lg overflow-auto max-h-96 text-sm text-gray-300">
          {messageDetails}
        </pre>
        <button
          onClick={() => setIsModalOpen(false)}
          className="mt-4 w-full bg-red-500 text-white py-2 rounded-lg hover:bg-red-600 transition-colors"
        >
          Close
        </button>
      </Modal>
    </div>
  );
};

export default FireReportForm;
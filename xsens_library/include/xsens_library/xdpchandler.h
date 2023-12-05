#ifndef XDPC_HANDLER
#define XDPC_HANDLER

#include <movelladot_pc_sdk.h>
#include <xscommon/xsens_mutex.h>
#include <list>

class XdpcHandler : public XsDotCallback
{
public:
	XdpcHandler(size_t maxBufferSize = 5);
	virtual ~XdpcHandler() noexcept;

	bool initialize();
	void scanForDots();
	void detectUsbDevices();
	void connectDots();
	void cleanup();

	XsDotConnectionManager* manager() const;

	XsPortInfoArray detectedDots() const;
	std::list<XsDotDevice*> connectedDots() const;
	std::list<XsDotUsbDevice*> connectedUsbDots() const;
	bool errorReceived() const;
	bool exportDone() const;
	bool updateDone() const;
	void resetUpdateDone();
	bool recordingStopped() const;
	void resetRecordingStopped();
	bool packetsAvailable() const;
	bool packetAvailable(const XsString& bluetoothAddress) const;
	XsDataPacket getNextPacket(const XsString& bluetoothAddress);
	int packetsReceived() const;
	void addDeviceToProgressBuffer(XsString bluetoothAddress);
	int progress(XsString bluetoothAddress);

protected:
	void onAdvertisementFound(const XsPortInfo* portInfo) override;
	void onBatteryUpdated(XsDotDevice* device, int batteryLevel, int chargingStatus) override;
	void onLiveDataAvailable(XsDotDevice* device, const XsDataPacket* packet) override;
	void onProgressUpdated(XsDotDevice* device, int current, int total, const XsString* identifier) override;
	void onDeviceUpdateDone(const XsPortInfo* portInfo, XsDotFirmwareUpdateResult result) override;
	void onError(XsResultValue result, const XsString* error) override;
	void onRecordingStopped(XsDotDevice* device) override;
	void onDeviceStateChanged(XsDotDevice* device, XsDeviceState newState, XsDeviceState oldState) override;
	void onButtonClicked(XsDotDevice* device, uint32_t timestamp) override;
	void onProgressUpdated(XsDotUsbDevice* device, int current, int total, const XsString* identifier) override;
	void onRecordedDataAvailable(XsDotUsbDevice* device, const XsDataPacket* packet) override;
	void onRecordedDataAvailable(XsDotDevice* device, const XsDataPacket* packet) override;
	void onRecordedDataDone(XsDotUsbDevice* device) override;
	void onRecordedDataDone(XsDotDevice* device) override;

private:
	void outputDeviceProgress() const;

	XsDotConnectionManager* m_manager = nullptr;

	mutable xsens::Mutex m_mutex;
	bool m_errorReceived = false;
	bool m_updateDone = false;
	bool m_recordingStopped = false;
	bool m_exportDone = false;
	bool m_closing = false;
	int m_progressCurrent = 0;
	int m_progressTotal = 0;
	int m_packetsReceived = 0;
	XsPortInfoArray m_detectedDots;
	std::list<XsDotDevice*> m_connectedDots;
	std::list<XsDotUsbDevice*> m_connectedUsbDots;

	size_t m_maxNumberOfPacketsInBuffer;
	std::map<XsString, size_t> m_numberOfPacketsInBuffer;
	std::map<XsString, std::list<XsDataPacket>> m_packetBuffer;
	std::map<XsString, int> m_progressBuffer;
};

#endif


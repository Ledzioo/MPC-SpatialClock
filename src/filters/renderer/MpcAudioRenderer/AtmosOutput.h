/*
 * AtmosOutput — MPC-BE Atmos Edition
 *
 * Decodes TrueHD (Atmos or not) received packed as IEC 61937 / MAT, and E-AC-3
 * (Atmos or not) received as IEC 61937, and plays it
 * through Windows Spatial Sound ("Dolby Atmos for home theater"): each element
 * of the track becomes a positioned audio object. TrueHD decoding is done by
 * thdatmos.dll (based on truehdd, Apache 2.0 license); E-AC-3 decoding uses
 * OpenJOC (Apache 2.0), which reconstructs the Atmos objects.
 * Also plays PCM with height channels (DTS:X decoded by MPC-BE, 7.1.4 files):
 * each channel becomes an object fixed at its speaker position.
 *
 * ReClock-style sync: when enabled, playback is sped up or slowed down by a
 * factor close to 1 (resampling) so that the film follows the real display
 * refresh rate exactly.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#pragma once

#include <windows.h>
#include <mmdeviceapi.h>
#include <audioclient.h>
#include <spatialaudioclient.h>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <atomic>
#include <mutex>
#include <vector>
#include "SyncTools.h"

struct AVCodecContext;
struct AVFrame;
struct AVPacket;

// Diagnostic log: SpatialClock.log, next to the module (.ax).
void AtmosLog(const wchar_t* fmt, ...);
// Diagnostic log on/off (renderer option, off by default).
void AtmosLogEnable(bool bEnable);

class CAtmosOutput
{
public:
	CAtmosOutput();
	~CAtmosOutput();

	// Is thdatmos.dll present next to the module and loaded?
	static bool LibraryAvailable();

	// Opens the spatial stream on the device ("Dolby Atmos for home theater" must be
	// active) and starts rendering (silence while SetRunning(false)).
	HRESULT Open(IMMDevice* pDevice);
	void    Close();
	bool    IsOpen() const { return m_pStream != nullptr; }

	// One IEC 61937 TrueHD burst. Blocks while more than ~1 s is queued, unless
	// hAbort1/hAbort2 are signaled (stop, flush).
	HRESULT Receive(const BYTE* pData, long size, REFERENCE_TIME rtStart, bool bDiscontinuity,
					HANDLE hAbort1, HANDLE hAbort2);
	// Interleaved float PCM at 48 kHz; channel order and positions from the WAVE channel mask.
	HRESULT ReceivePcm(const float* pPcm, uint32_t samples, uint32_t channels, uint32_t channelMask,
					   REFERENCE_TIME rtStart, bool bDiscontinuity, HANDLE hAbort);

	void Flush();                     // clears the queue and resets the decoder
	void SetRunning(bool bRunning);   // false: render silence
	void SetVolume(float volume);     // global linear gain
	void SetEac3Gain(float dB);        // level trim for E-AC-3 Atmos (objects) only

	// Decode-only mode (no spatial stream): TrueHD / E-AC-3 decoded to 8-channel (7.1) float PCM at 48 kHz,
	// for the normal WASAPI path when the device cannot play objects or the bitstream.
	static constexpr uint32_t PCM71_CHANNELS = 8; // FL FR FC LFE BL BR SL SR
	bool OpenDecoder();
	bool DecodeToPcm71(const BYTE* pData, long size, std::vector<float>& pcm, uint32_t& frames);

	// Playback speed (1.0 = normal), applied by resampling.
	void   SetSpeed(double speed);
	double GetSpeed();

	IAudioClock*   GetAudioClock() const { return m_pAudioClock; }
	// Audio clock rate relative to the system clock (QPC): audio seconds per
	// system second (about 1 +/- a few ppm), and the measurement span in seconds.
	bool           GetAudioClockRate(double& rate, double& seconds);
	REFERENCE_TIME GetCurrentRenderedTime();
	size_t         QueuedSamples();
	unsigned       GetDynamicObjectCount() const { return m_nMaxDynamic; }
	// Decoded stream: bit 0 TrueHD, bit 1 E-AC-3, bit 2 Atmos objects (0 if unknown).
	uint32_t       GetStreamInfo();
	std::atomic<uint32_t> m_nStreamInfo{0};

	// Called by the render thread when timestamps jump (like the original renderer).
	std::function<void(REFERENCE_TIME)> OnClockOffset;
	// Called by the render thread about once a second (speed control).
	std::function<void()> OnTick;

	struct Element { uint32_t kind, active; float x, y, z, gain; };
	struct Frame {
		uint64_t sample_pos; uint32_t samples, channels, sample_rate;
		float pcm[160][16];
		Element elements[16];
	};

private:
	struct QueuedFrame {
		REFERENCE_TIME rtStart;
		std::unique_ptr<Frame> f;
	};
	// Buffered input segment: metadata valid from sample `start`.
	struct Segment {
		uint64_t       start;
		uint32_t       len;
		uint32_t       channels;
		REFERENCE_TIME rtStart;
		Element        elements[16];
	};

	bool  WaitForSpace(HANDLE hAbort1, HANDLE hAbort2); // false if aborted
	void  QueueFrame(std::unique_ptr<Frame>& frame, REFERENCE_TIME rtStart, bool& bDiscontinuity);
	void  QueueResampled(std::unique_ptr<Frame>& frame, REFERENCE_TIME rtStart, bool& bDiscontinuity);
	// Sample rate conversion to 48 kHz (tracks at other rates)
	CVarResampler      m_rateRs;
	uint32_t           m_rateRsRate = 0;
	std::vector<float> m_rateIn, m_rateOut;
	static DWORD WINAPI ThreadProc(LPVOID p) { return static_cast<CAtmosOutput*>(p)->RenderThread(); }
	DWORD RenderThread();
	void  RenderPeriod();
	bool  PullFrame();                             // queue -> input buffer (lock held)
	float Sample(unsigned c, int64_t index) const; // input sample (0 outside the buffer)
	const Segment* SegmentAt(uint64_t index) const;
	void  ResetBuffers();

	// E-AC-3 without Atmos objects is decoded by FFmpeg (the OpenJOC core decoder does not
	// support the usual coupling / spectral extension tools). One IEC 61937 burst in, frames out.
	bool  FFOpen();
	void  FFClose();
	void  FFDecodeEac3(const BYTE* pData, long size, std::vector<std::unique_ptr<Frame>>& out);
	AVCodecContext* m_pFFCtx = nullptr;
	AVFrame*        m_pFFFrame = nullptr;
	AVPacket*       m_pFFPkt = nullptr;
	bool            m_bFFFailed = false;
	std::atomic<float> m_fEac3Gain{1.0f};

	ISpatialAudioClient*             m_pClient = nullptr;
	ISpatialAudioObjectRenderStream* m_pStream = nullptr;
	IAudioClock*                     m_pAudioClock = nullptr;
	ISpatialAudioObject*             m_pObjects[16] = {};
	uint32_t                         m_ObjectKind[16] = {};
	unsigned                         m_nMaxDynamic = 0;

	HANDLE m_hBufferEvent = nullptr;
	HANDLE m_hStopEvent = nullptr;
	HANDLE m_hSpaceEvent = nullptr;   // signaled when the queue drains
	HANDLE m_hThread = nullptr;

	std::mutex              m_mutex;
	std::deque<QueuedFrame> m_queue;
	size_t                  m_nQueuedSamples = 0;

	// Input buffer: m_in[c][k] = sample at index m_base + k
	std::vector<float>  m_in[16];
	uint64_t            m_base = 0;
	uint64_t            m_end = 0;      // index after the last buffered sample
	double              m_pos = 0.0;    // read position (in input samples)
	std::deque<Segment> m_segments;
	REFERENCE_TIME      m_rtNextQueued = 0;

	Element m_State[16] = {};
	bool    m_bHaveState = false;

	void*          m_pDecoder = nullptr; // thdatmos context
	REFERENCE_TIME m_rtNextDecoded = INVALID_TIME_VALUE; // anchor (segment start)
	uint64_t       m_nDecodedSinceAnchor = 0;
	REFERENCE_TIME m_rtCurrentRendered = 0;
	bool           m_bRunning = false;
	float          m_fVolume = 1.0f;
	double         m_dSpeed = 1.0;
	DWORD          m_dwLastTick = 0;

	// Audio clock vs system clock measurement
	CClockRateMeter m_ClockMeter;

	static constexpr REFERENCE_TIME INVALID_TIME_VALUE = INT64_MIN;
};

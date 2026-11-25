/*
 * AtmosOutput — MPC-BE Atmos Edition (see AtmosOutput.h)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "stdafx.h"
#include "AtmosOutput.h"
#include <algorithm>
#include <cstring>
#include <cmath>
#include <cstdarg>
#include <cstdio>

#pragma warning(push)
#pragma warning(disable: 4005)
extern "C" {
	#include <ExtLib/ffmpeg/libavcodec/avcodec.h>
	#include <ExtLib/ffmpeg/libavutil/channel_layout.h>
	#include <ExtLib/ffmpeg/libavutil/frame.h>
}
#pragma warning(pop)

// ---- thdatmos.dll ----
namespace
{
	typedef void* (*PFN_thd_create)();
	typedef void  (*PFN_thd_reset)(void*);
	typedef int   (*PFN_thd_push_iec61937)(void*, const uint8_t*, size_t);
	typedef int   (*PFN_thd_pull)(void*, CAtmosOutput::Frame*);
	typedef void  (*PFN_thd_close)(void*);
	typedef uint32_t (*PFN_thd_info)(const void*);
	typedef const char* (*PFN_thd_error)(const void*);

	static int s_moduleAnchor = 0;

	struct ThdLib {
		HMODULE hLib = nullptr;
		bool tried = false;
		PFN_thd_create        create = nullptr;
		PFN_thd_reset         reset = nullptr;
		PFN_thd_push_iec61937 push = nullptr;
		PFN_thd_pull          pull = nullptr;
		PFN_thd_close         close = nullptr;
		PFN_thd_info          info = nullptr; // optional
		PFN_thd_error         error = nullptr; // optional (diagnostics)

		bool Load() {
			if (tried) {
				return hLib != nullptr;
			}
			tried = true;
			wchar_t path[MAX_PATH] = {};
			// thdatmos.dll is loaded from the folder of the module containing this code (.ax or .exe).
			HMODULE hSelf = nullptr;
			GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
							   (LPCWSTR)&s_moduleAnchor, &hSelf);
			GetModuleFileNameW(hSelf, path, MAX_PATH);
			if (wchar_t* p = wcsrchr(path, L'\\')) {
				wcscpy_s(p + 1, MAX_PATH - (p + 1 - path), L"thdatmos.dll");
			}
			hLib = LoadLibraryW(path);
			if (!hLib) {
				AtmosLog(L"thdatmos.dll not found or unreadable: %s (error %u)", path, GetLastError());
				return false;
			}
			create = (PFN_thd_create)GetProcAddress(hLib, "thd_create");
			reset  = (PFN_thd_reset)GetProcAddress(hLib, "thd_reset");
			push   = (PFN_thd_push_iec61937)GetProcAddress(hLib, "thd_push_iec61937");
			pull   = (PFN_thd_pull)GetProcAddress(hLib, "thd_pull");
			close  = (PFN_thd_close)GetProcAddress(hLib, "thd_close");
			info   = (PFN_thd_info)GetProcAddress(hLib, "thd_info");
			error  = (PFN_thd_error)GetProcAddress(hLib, "thd_error");
			if (!create || !reset || !push || !pull || !close) {
				AtmosLog(L"thdatmos.dll: missing exports");
				FreeLibrary(hLib);
				hLib = nullptr;
			} else {
				AtmosLog(L"thdatmos.dll loaded: %s", path);
			}
			return hLib != nullptr;
		}
	};
	ThdLib g_thd;

	constexpr uint32_t KIND_LFE = 1;
	constexpr REFERENCE_TIME UNITS_PER_SEC = 10000000;
	constexpr size_t MAX_QUEUE_SAMPLES = 48000;   // ~1 s ahead
	constexpr uint32_t SAMPLE_RATE = 48000;

	inline REFERENCE_TIME SamplesToTime(uint64_t n) { return (REFERENCE_TIME)(n * UNITS_PER_SEC / SAMPLE_RATE); }

	template <class T> void SafeRelease(T*& p) { if (p) { p->Release(); p = nullptr; } }

	constexpr int KERNEL_HALF = SincKernel::HALF;
	constexpr int KERNEL_PHASES = SincKernel::PHASES;
}

static std::atomic<bool> s_logEnabled{false};

void AtmosLogEnable(bool bEnable)
{
	s_logEnabled = bEnable;
}

void AtmosLog(const wchar_t* fmt, ...)
{
	if (!s_logEnabled) {
		return; // option off: nothing is written
	}
	static std::mutex s_logMutex;
	static wchar_t s_path[MAX_PATH] = {};
	std::lock_guard<std::mutex> lock(s_logMutex);
	if (!s_path[0]) {
		HMODULE hSelf = nullptr;
		GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
						   (LPCWSTR)&s_moduleAnchor, &hSelf);
		GetModuleFileNameW(hSelf, s_path, MAX_PATH);
		if (wchar_t* p = wcsrchr(s_path, L'\\')) {
			wcscpy_s(p + 1, MAX_PATH - (p + 1 - s_path), L"SpatialClock.log");
		}
	}
	FILE* f = nullptr;
	if (_wfopen_s(&f, s_path, L"a, ccs=UTF-8") != 0 || !f) {
		return;
	}
	SYSTEMTIME st;
	GetLocalTime(&st);
	fwprintf(f, L"%02u:%02u:%02u.%03u  ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
	va_list ap;
	va_start(ap, fmt);
	vfwprintf(f, fmt, ap);
	va_end(ap);
	fwprintf(f, L"\n");
	fclose(f);
}

CAtmosOutput::CAtmosOutput()
{
	m_hStopEvent  = CreateEventW(nullptr, TRUE, FALSE, nullptr);
	m_hSpaceEvent = CreateEventW(nullptr, FALSE, FALSE, nullptr);
}

CAtmosOutput::~CAtmosOutput()
{
	Close();
	CloseHandle(m_hStopEvent);
	CloseHandle(m_hSpaceEvent);
}

bool CAtmosOutput::LibraryAvailable()
{
	return g_thd.Load();
}

HRESULT CAtmosOutput::Open(IMMDevice* pDevice)
{
	if (IsOpen()) {
		return S_OK;
	}
	if (!pDevice) {
		AtmosLog(L"Open: no device");
		return E_FAIL;
	}

	HRESULT hr = pDevice->Activate(__uuidof(ISpatialAudioClient), CLSCTX_INPROC_SERVER, nullptr, (void**)&m_pClient);
	if (FAILED(hr)) {
		AtmosLog(L"Open: ISpatialAudioClient unavailable (0x%08x)", hr);
		return hr;
	}
	hr = m_pClient->IsSpatialAudioStreamAvailable(__uuidof(ISpatialAudioObjectRenderStream), nullptr);
	if (FAILED(hr)) {
		AtmosLog(L"Open: no spatial sound format active (0x%08x)", hr);
		Close();
		return hr;
	}

	UINT32 maxDyn = 0;
	m_pClient->GetMaxDynamicObjectCount(&maxDyn);
	m_nMaxDynamic = std::min<UINT32>(maxDyn, 16);
	AtmosLog(L"Open: %u dynamic objects available", maxDyn);
	if (m_nMaxDynamic < 15) { // a Blu-ray Atmos track has up to 15 objects + LFE
		AtmosLog(L"Open: not enough dynamic objects");
		Close();
		return E_FAIL;
	}

	WAVEFORMATEX wf = {};
	wf.wFormatTag      = WAVE_FORMAT_IEEE_FLOAT;
	wf.nChannels       = 1;
	wf.nSamplesPerSec  = SAMPLE_RATE;
	wf.wBitsPerSample  = 32;
	wf.nBlockAlign     = 4;
	wf.nAvgBytesPerSec = SAMPLE_RATE * 4;
	hr = m_pClient->IsAudioObjectFormatSupported(&wf);
	if (FAILED(hr)) {
		AtmosLog(L"Open: 48 kHz float format refused (0x%08x)", hr);
		Close();
		return hr;
	}

	m_hBufferEvent = CreateEventW(nullptr, FALSE, FALSE, nullptr);

	SpatialAudioObjectRenderStreamActivationParams p = {};
	p.ObjectFormat          = &wf;
	p.StaticObjectTypeMask  = AudioObjectType_LowFrequency;
	p.MinDynamicObjectCount = 0;
	p.MaxDynamicObjectCount = m_nMaxDynamic;
	p.Category              = AudioCategory_Movie;
	p.EventHandle           = m_hBufferEvent;
	p.NotifyObject          = nullptr;

	PROPVARIANT act;
	PropVariantInit(&act);
	act.vt             = VT_BLOB;
	act.blob.cbSize    = sizeof(p);
	act.blob.pBlobData = (BYTE*)&p;
	hr = m_pClient->ActivateSpatialAudioStream(&act, __uuidof(ISpatialAudioObjectRenderStream), (void**)&m_pStream);
	if (FAILED(hr)) {
		AtmosLog(L"Open: ActivateSpatialAudioStream failed (0x%08x)", hr);
		Close();
		return hr;
	}

	hr = m_pStream->GetService(__uuidof(IAudioClock), (void**)&m_pAudioClock);
	if (SUCCEEDED(hr)) {
		m_ClockMeter.Reset();
	}
	if (FAILED(hr)) {
		AtmosLog(L"Open: IAudioClock unavailable (0x%08x)", hr);
		Close();
		return hr;
	}

	m_pDecoder = LibraryAvailable() ? g_thd.create() : nullptr; // TrueHD only; PCM needs no decoder

	ResetEvent(m_hStopEvent);
	hr = m_pStream->Start();
	if (FAILED(hr)) {
		AtmosLog(L"Open: Start failed (0x%08x)", hr);
		Close();
		return hr;
	}
	AtmosLog(L"Open: spatial stream opened");
	m_hThread = CreateThread(nullptr, 0, ThreadProc, this, 0, nullptr);

	return S_OK;
}

void CAtmosOutput::Close()
{
	if (m_hThread) {
		SetEvent(m_hStopEvent);
		WaitForSingleObject(m_hThread, 2000);
		CloseHandle(m_hThread);
		m_hThread = nullptr;
	}
	if (m_pStream) {
		m_pStream->Stop();
	}
	for (auto& o : m_pObjects) {
		SafeRelease(o);
	}
	SafeRelease(m_pAudioClock);
	SafeRelease(m_pStream);
	SafeRelease(m_pClient);
	if (m_hBufferEvent) {
		CloseHandle(m_hBufferEvent);
		m_hBufferEvent = nullptr;
	}
	if (m_pDecoder) {
		g_thd.close(m_pDecoder);
		m_pDecoder = nullptr;
	}
	FFClose();
	m_bFFFailed = false;
	std::lock_guard<std::mutex> lock(m_mutex);
	m_queue.clear();
	m_nQueuedSamples = 0;
	ResetBuffers();
	m_bHaveState = false;
}

static CAtmosOutput::Element SpeakerElement(DWORD speaker, bool bSurroundAtSide);

// IEC 61937 E-AC-3 burst -> raw E-AC-3 (same unpacking as thdatmos: 16-bit words, byte-swapped).
static bool UnpackEac3Burst(const BYTE* b, size_t size, std::vector<uint8_t>& out)
{
	if (size < 8) {
		return false;
	}
	auto w = [&](size_t i) { return (uint16_t)(b[i] | (b[i + 1] << 8)); };
	if (w(0) != 0xF872 || w(2) != 0x4E1F || (w(4) & 0x7F) != 0x15) {
		return false;
	}
	const size_t avail = size - 8;
	const size_t len   = std::min<size_t>(w(6), avail);
	const size_t even  = std::min<size_t>(len + 1, avail) & ~(size_t)1;
	out.assign(even, 0);
	for (size_t i = 0; i + 1 < even; i += 2) {
		out[i]     = b[8 + i + 1];
		out[i + 1] = b[8 + i];
	}
	out.resize(std::min(len, even));
	// Packers that do not swap: the syncword then reads 77 0B.
	if (out.size() >= 2 && out[0] == 0x77 && out[1] == 0x0B) {
		for (size_t i = 0; i + 1 < out.size(); i += 2) {
			std::swap(out[i], out[i + 1]);
		}
	}
	return !out.empty();
}

bool CAtmosOutput::FFOpen()
{
	const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_EAC3);
	if (!codec) {
		AtmosLog(L"FFmpeg: E-AC-3 decoder not found");
		m_bFFFailed = true;
		return false;
	}
	m_pFFCtx   = avcodec_alloc_context3(codec);
	m_pFFFrame = av_frame_alloc();
	m_pFFPkt   = av_packet_alloc();
	if (!m_pFFCtx || !m_pFFFrame || !m_pFFPkt || avcodec_open2(m_pFFCtx, codec, nullptr) < 0) {
		AtmosLog(L"FFmpeg: E-AC-3 decoder initialization failed");
		FFClose();
		m_bFFFailed = true;
		return false;
	}
	AtmosLog(L"FFmpeg: E-AC-3 decoder opened");
	return true;
}

void CAtmosOutput::FFClose()
{
	if (m_pFFCtx) {
		avcodec_free_context(&m_pFFCtx);
	}
	if (m_pFFFrame) {
		av_frame_free(&m_pFFFrame);
	}
	if (m_pFFPkt) {
		av_packet_free(&m_pFFPkt);
	}
}

void CAtmosOutput::FFDecodeEac3(const BYTE* pData, long size, std::vector<std::unique_ptr<Frame>>& out)
{
	std::vector<uint8_t> es;
	if (!UnpackEac3Burst(pData, (size_t)size, es)) {
		return;
	}
	if (!m_pFFCtx && (m_bFFFailed || !FFOpen())) {
		return;
	}

	const size_t esSize = es.size();
	es.resize(esSize + AV_INPUT_BUFFER_PADDING_SIZE, 0);
	m_pFFPkt->data = es.data();
	m_pFFPkt->size = (int)esSize;
	const int ret = avcodec_send_packet(m_pFFCtx, m_pFFPkt);
	m_pFFPkt->data = nullptr;
	m_pFFPkt->size = 0;
	if (ret < 0 && ret != AVERROR(EAGAIN)) {
		static int s_errLogged = 0;
		if (s_errLogged++ < 5) {
			AtmosLog(L"FFmpeg: avcodec_send_packet failed (%d)", ret);
		}
	}

	while (avcodec_receive_frame(m_pFFCtx, m_pFFFrame) >= 0) {
		const AVFrame* f = m_pFFFrame;
		const int nbCh   = f->ch_layout.nb_channels;
		const uint32_t nch = (uint32_t)std::min(nbCh, 16);
		const bool bFloat  = (f->format == AV_SAMPLE_FMT_FLTP || f->format == AV_SAMPLE_FMT_FLT);
		if (!nch || f->nb_samples <= 0 || !bFloat) {
			static int s_fmtLogged = 0;
			if (!bFloat && s_fmtLogged++ < 3) {
				AtmosLog(L"FFmpeg: unexpected sample format %d", f->format);
			}
			av_frame_unref(m_pFFFrame);
			continue;
		}
		const bool bPlanar = (f->format == AV_SAMPLE_FMT_FLTP);

		// Fixed position of each channel.
		Element elements[16] = {};
		int chan[16];
		DWORD mask = 0;
		for (uint32_t c = 0; c < nch; c++) {
			chan[c] = (int)av_channel_layout_channel_from_index(&f->ch_layout, c);
			if (chan[c] >= 0 && chan[c] <= 17) { // AVChannel 0..17 = WAVE speaker bits 0..17
				mask |= 1u << chan[c];
			}
		}
		const bool bSurroundAtSide = (mask & (SPEAKER_BACK_LEFT | SPEAKER_SIDE_LEFT)) == SPEAKER_BACK_LEFT; // 5.1 "back"
		for (uint32_t c = 0; c < nch; c++) {
			if (chan[c] >= 0 && chan[c] <= 17) {
				elements[c] = SpeakerElement(1u << chan[c], bSurroundAtSide);
			}
		}

		const uint32_t total = (uint32_t)f->nb_samples;
		for (uint32_t done = 0; done < total;) {
			const uint32_t k = std::min<uint32_t>(total - done, 160);
			auto fr = std::make_unique<Frame>();
			fr->sample_pos  = 0;
			fr->samples     = k;
			fr->channels    = nch;
			fr->sample_rate = (uint32_t)f->sample_rate;
			memcpy(fr->elements, elements, sizeof(elements));
			for (uint32_t s = 0; s < k; s++) {
				for (uint32_t c = 0; c < nch; c++) {
					fr->pcm[s][c] = bPlanar
						? ((const float*)f->extended_data[c])[done + s]
						: ((const float*)f->data[0])[(size_t)(done + s) * nbCh + c];
				}
			}
			out.push_back(std::move(fr));
			done += k;
		}
		av_frame_unref(m_pFFFrame);
	}
}

HRESULT CAtmosOutput::Receive(const BYTE* pData, long size, REFERENCE_TIME rtStart, bool bDiscontinuity,
							  HANDLE hAbort1, HANDLE hAbort2)
{
	if (!m_pDecoder) {
		return E_FAIL;
	}

	if (!WaitForSpace(hAbort1, hAbort2)) {
		return S_FALSE;
	}

	if (!g_thd.push(m_pDecoder, pData, (size_t)size)) {
		static int s_badLogged = 0;
		if (s_badLogged++ < 3) {
			AtmosLog(L"Receive: unrecognized burst (%ld bytes, starts %02x %02x %02x %02x %02x)", size,
					 size > 0 ? pData[0] : 0, size > 1 ? pData[1] : 0, size > 2 ? pData[2] : 0, size > 3 ? pData[3] : 0, size > 4 ? pData[4] : 0);
		}
		return S_FALSE; // not packed TrueHD
	}

	if (g_thd.info) {
		const uint32_t info = g_thd.info(m_pDecoder); // read on this thread only (the decoder is not shared)
		if (info != m_nStreamInfo) {
			AtmosLog(L"Receive: stream info 0x%x (bit0 TrueHD, bit1 E-AC-3, bit2 Atmos objects)", info);
			if (info & 2) {
				AtmosLog((info & 4) ? L"Receive: E-AC-3 with Atmos objects -> thdatmos (OpenJOC), trim %.1f dB"
									: L"Receive: E-AC-3 without Atmos objects -> FFmpeg decoder",
						 20.0 * log10((double)m_fEac3Gain.load()));
			}
		}
		m_nStreamInfo = info;
	}

	// Diagnostics: how many bursts went in and how many frames came out.
	static uint64_t s_bursts = 0, s_frames = 0;
	s_bursts++;

	const uint32_t info = m_nStreamInfo.load();
	const bool bFFmpeg     = (info & 2) && !(info & 4); // E-AC-3 without Atmos objects
	const bool bEac3Atmos  = (info & 2) && (info & 4);
	const float fGain      = bEac3Atmos ? m_fEac3Gain.load() : 1.0f;

	std::vector<std::unique_ptr<Frame>> frames;
	{
		auto f = std::make_unique<Frame>();
		while (g_thd.pull(m_pDecoder, f.get()) == 1) {
			if (bFFmpeg) {
				continue; // the thdatmos core decoder output is not used, FFmpeg decodes this stream
			}
			if (fGain != 1.0f) {
				for (uint32_t s = 0; s < f->samples && s < 160; s++) {
					for (uint32_t c = 0; c < f->channels && c < 16; c++) {
						f->pcm[s][c] *= fGain;
					}
				}
			}
			frames.push_back(std::move(f));
			f = std::make_unique<Frame>();
		}
	}
	if (bFFmpeg) {
		FFDecodeEac3(pData, size, frames);
	}

	for (auto& frame : frames) {
		if (!frame->samples) {
			continue;
		}
		if (s_frames < 5 || (s_frames % 2000) == 0) {
			float peak = 0.0f;
			for (uint32_t s = 0; s < frame->samples; s++) {
				for (uint32_t c = 0; c < frame->channels && c < 16; c++) {
					peak = std::max(peak, std::fabs(frame->pcm[s][c]));
				}
			}
			int active = 0;
			for (uint32_t c = 0; c < frame->channels && c < 16; c++) {
				active += frame->elements[c].active ? 1 : 0;
			}
			AtmosLog(L"Receive: frame #%llu (burst #%llu): %u samples, %u ch, %u Hz, peak %.4f, %d active elements",
					 (unsigned long long)s_frames, (unsigned long long)s_bursts, frame->samples, frame->channels, frame->sample_rate, peak, active);
		}
		s_frames++;
		if (frame->sample_rate != SAMPLE_RATE) {
			if (!frame->sample_rate || frame->sample_rate > 8 * SAMPLE_RATE) {
				continue;
			}
			// The spatial stream runs at 48 kHz (rare tracks at other rates). First an integer
			// decimation by averaging (96 -> 48, 88.2 -> 44.1 kHz...), then, unless already
			// at 48 kHz, sinc resampling up to 48 kHz.
			const uint32_t f = (frame->sample_rate + SAMPLE_RATE - 1) / SAMPLE_RATE;
			const uint32_t n = frame->samples / f;
			for (uint32_t s = 0; s < n; s++) {
				for (uint32_t c = 0; c < frame->channels && c < 16; c++) {
					float acc = 0.0f;
					for (uint32_t k = 0; k < f; k++) {
						acc += frame->pcm[s * f + k][c];
					}
					frame->pcm[s][c] = acc / (float)f;
				}
			}
			frame->samples = n;
			frame->sample_rate /= f;
			if (!n) {
				continue;
			}
			if (frame->sample_rate != SAMPLE_RATE) {
				QueueResampled(frame, rtStart, bDiscontinuity);
				continue;
			}
		}
		QueueFrame(frame, rtStart, bDiscontinuity);
	}

	if ((s_bursts == 50 || s_bursts == 500) && s_frames == 0) {
		const char* err = g_thd.error ? g_thd.error(m_pDecoder) : nullptr;
		AtmosLog(L"Receive: %llu bursts pushed, no frames produced; last decoder error: %S", (unsigned long long)s_bursts, (err && *err) ? err : "(none)");
	}

	return S_OK;
}

void CAtmosOutput::QueueResampled(std::unique_ptr<Frame>& frame, REFERENCE_TIME rtStart, bool& bDiscontinuity)
{
	const uint32_t ch = std::min<uint32_t>(frame->channels, 16);
	if (!ch) {
		return;
	}
	if (bDiscontinuity || frame->sample_rate != m_rateRsRate || ch != m_rateRs.Channels()) {
		m_rateRs.Reset(ch);
		m_rateRsRate = frame->sample_rate;
	}
	m_rateIn.resize((size_t)frame->samples * ch);
	for (uint32_t s = 0; s < frame->samples; s++) {
		for (uint32_t c = 0; c < ch; c++) {
			m_rateIn[(size_t)s * ch + c] = frame->pcm[s][c];
		}
	}
	double firstPos = 0.0;
	const size_t n = m_rateRs.Process(m_rateIn.data(), frame->samples, (double)frame->sample_rate / SAMPLE_RATE, m_rateOut, firstPos);
	Element elements[16];
	memcpy(elements, frame->elements, sizeof(elements));
	size_t done = 0;
	while (done < n) {
		auto out = std::make_unique<Frame>();
		const uint32_t k = (uint32_t)std::min<size_t>(n - done, 160);
		out->sample_pos  = 0;
		out->samples     = k;
		out->channels    = ch;
		out->sample_rate = SAMPLE_RATE;
		memcpy(out->elements, elements, sizeof(elements));
		for (uint32_t s = 0; s < k; s++) {
			for (uint32_t c = 0; c < ch; c++) {
				out->pcm[s][c] = m_rateOut[(done + s) * ch + c];
			}
		}
		QueueFrame(out, rtStart, bDiscontinuity);
		done += k;
	}
}

bool CAtmosOutput::WaitForSpace(HANDLE hAbort1, HANDLE hAbort2)
{
	// Back-pressure: stay at most about one second ahead of playback.
	for (;;) {
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			if (m_nQueuedSamples < MAX_QUEUE_SAMPLES) {
				return true;
			}
		}
		HANDLE h[3] = { m_hSpaceEvent, hAbort1, hAbort2 };
		const DWORD n = hAbort2 ? 3 : (hAbort1 ? 2 : 1);
		const DWORD r = WaitForMultipleObjects(n, h, FALSE, 100);
		if (r == WAIT_OBJECT_0 + 1 || r == WAIT_OBJECT_0 + 2) {
			return false;
		}
	}
}

void CAtmosOutput::QueueFrame(std::unique_ptr<Frame>& frame, REFERENCE_TIME rtStart, bool& bDiscontinuity)
{
	if (m_rtNextDecoded == INVALID_TIME_VALUE || bDiscontinuity) {
		// Timestamp anchor: start of the current input packet.
		m_rtNextDecoded = rtStart;
		m_nDecodedSinceAnchor = 0;
		bDiscontinuity = false;
	}
	const REFERENCE_TIME t = m_rtNextDecoded + SamplesToTime(m_nDecodedSinceAnchor);
	m_nDecodedSinceAnchor += frame->samples;

	std::lock_guard<std::mutex> lock(m_mutex);
	m_nQueuedSamples += frame->samples;
	m_queue.push_back({ t, std::move(frame) });
	frame = std::make_unique<Frame>();
}

// Fixed position of a WAVE speaker (DAMF: x left/right, y back/front, z floor/ceiling).
static CAtmosOutput::Element SpeakerElement(DWORD speaker, bool bSurroundAtSide)
{
	float x = 0, y = 0, z = 0;
	switch (speaker) {
		case SPEAKER_LOW_FREQUENCY:      return { KIND_LFE, 1, 0, 0, 0, 1.0f };
		case SPEAKER_FRONT_LEFT:         x = -1.0f; y =  1.0f; break;
		case SPEAKER_FRONT_RIGHT:        x =  1.0f; y =  1.0f; break;
		case SPEAKER_FRONT_CENTER:       x =  0.0f; y =  1.0f; break;
		case SPEAKER_BACK_LEFT:          x = -1.0f; y = bSurroundAtSide ? 0.0f : -1.0f; break;
		case SPEAKER_BACK_RIGHT:         x =  1.0f; y = bSurroundAtSide ? 0.0f : -1.0f; break;
		case SPEAKER_FRONT_LEFT_OF_CENTER:  x = -0.5f; y = 1.0f; break;
		case SPEAKER_FRONT_RIGHT_OF_CENTER: x =  0.5f; y = 1.0f; break;
		case SPEAKER_BACK_CENTER:        x =  0.0f; y = -1.0f; break;
		case SPEAKER_SIDE_LEFT:          x = -1.0f; y =  0.0f; break;
		case SPEAKER_SIDE_RIGHT:         x =  1.0f; y =  0.0f; break;
		case SPEAKER_TOP_CENTER:         x =  0.0f; y =  0.0f; z = 1.0f; break;
		case SPEAKER_TOP_FRONT_LEFT:     x = -1.0f; y =  1.0f; z = 1.0f; break;
		case SPEAKER_TOP_FRONT_CENTER:   x =  0.0f; y =  1.0f; z = 1.0f; break;
		case SPEAKER_TOP_FRONT_RIGHT:    x =  1.0f; y =  1.0f; z = 1.0f; break;
		case SPEAKER_TOP_BACK_LEFT:      x = -1.0f; y = -1.0f; z = 1.0f; break;
		case SPEAKER_TOP_BACK_CENTER:    x =  0.0f; y = -1.0f; z = 1.0f; break;
		case SPEAKER_TOP_BACK_RIGHT:     x =  1.0f; y = -1.0f; z = 1.0f; break;
		default: return { 0, 0, 0, 0, 0, 0.0f }; // unknown: muted
	}
	return { 0, 1, x, y, z, 1.0f };
}

HRESULT CAtmosOutput::ReceivePcm(const float* pPcm, uint32_t samples, uint32_t channels, uint32_t channelMask,
								 REFERENCE_TIME rtStart, bool bDiscontinuity, HANDLE hAbort)
{
	if (!IsOpen() || !channels || channels > 16) {
		return E_FAIL;
	}
	if (!WaitForSpace(hAbort, nullptr)) {
		return S_FALSE;
	}

	// Channels follow the mask bits in ascending order.
	Element elements[16] = {};
	const bool bSurroundAtSide = (channelMask & (SPEAKER_BACK_LEFT | SPEAKER_SIDE_LEFT)) == SPEAKER_BACK_LEFT; // 5.1 "back"
	unsigned c = 0;
	for (DWORD bit = 1; bit && c < channels; bit <<= 1) {
		if (channelMask & bit) {
			elements[c++] = SpeakerElement(bit, bSurroundAtSide);
		}
	}

	uint32_t done = 0;
	while (done < samples) {
		auto frame = std::make_unique<Frame>();
		const uint32_t n = std::min<uint32_t>(samples - done, 160);
		frame->sample_pos  = 0;
		frame->samples     = n;
		frame->channels    = channels;
		frame->sample_rate = SAMPLE_RATE;
		memcpy(frame->elements, elements, sizeof(elements));
		for (uint32_t s = 0; s < n; s++) {
			for (uint32_t ch = 0; ch < channels; ch++) {
				frame->pcm[s][ch] = pPcm[(size_t)(done + s) * channels + ch];
			}
		}
		const REFERENCE_TIME t = rtStart + SamplesToTime(done);
		QueueFrame(frame, t, bDiscontinuity);
		done += n;
	}
	return S_OK;
}

void CAtmosOutput::ResetBuffers()
{
	for (auto& v : m_in) {
		v.clear();
	}
	m_base = m_end = 0;
	m_pos = 0.0;
	m_segments.clear();
	m_rtNextQueued = 0;
}

void CAtmosOutput::Flush()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_queue.clear();
	m_nQueuedSamples = 0;
	ResetBuffers();
	m_rtNextDecoded = INVALID_TIME_VALUE;
	m_rtCurrentRendered = 0;
	m_rateRsRate = 0; // resampler restarts with the next frame
	if (m_pDecoder) {
		g_thd.reset(m_pDecoder);
	}
	if (m_pFFCtx) {
		avcodec_flush_buffers(m_pFFCtx);
	}
	SetEvent(m_hSpaceEvent);
}

uint32_t CAtmosOutput::GetStreamInfo()
{
	return m_nStreamInfo.load();
}

void CAtmosOutput::SetRunning(bool bRunning)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_bRunning = bRunning;
}

void CAtmosOutput::SetVolume(float volume)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_fVolume = volume;
}

void CAtmosOutput::SetEac3Gain(float dB)
{
	dB = std::min(std::max(dB, -30.0f), 30.0f);
	m_fEac3Gain = std::pow(10.0f, dB / 20.0f);
}

void CAtmosOutput::SetSpeed(double speed)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_dSpeed = speed;
}

double CAtmosOutput::GetSpeed()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_dSpeed;
}

const CAtmosOutput::Segment* CAtmosOutput::SegmentAt(uint64_t index) const
{
	const Segment* s = nullptr;
	for (const auto& seg : m_segments) {
		if (seg.start <= index) {
			s = &seg;
		} else {
			break;
		}
	}
	return s;
}

REFERENCE_TIME CAtmosOutput::GetCurrentRenderedTime()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	if (const Segment* s = SegmentAt((uint64_t)m_pos)) {
		return s->rtStart + (REFERENCE_TIME)((m_pos - (double)s->start) * UNITS_PER_SEC / SAMPLE_RATE);
	}
	if (!m_queue.empty()) {
		return m_queue.front().rtStart;
	}
	return m_rtCurrentRendered;
}

size_t CAtmosOutput::QueuedSamples()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	const double buffered = (double)m_end - m_pos;
	return m_nQueuedSamples + (buffered > 0 ? (size_t)buffered : 0);
}

DWORD CAtmosOutput::RenderThread()
{
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
	HANDLE h[2] = { m_hStopEvent, m_hBufferEvent };
	for (;;) {
		const DWORD r = WaitForMultipleObjects(2, h, FALSE, 200);
		if (r == WAIT_OBJECT_0) {
			break;
		}
		if (r == WAIT_OBJECT_0 + 1) {
			RenderPeriod();
		}
		m_ClockMeter.Poll(m_pAudioClock);
		const DWORD now = GetTickCount();
		if (now - m_dwLastTick >= 1000) {
			m_dwLastTick = now;
			if (OnTick) {
				OnTick();
			}
		}
	}
	return 0;
}

bool CAtmosOutput::GetAudioClockRate(double& rate, double& seconds)
{
	return m_ClockMeter.GetRate(rate, seconds);
}

bool CAtmosOutput::PullFrame()
{
	if (m_queue.empty()) {
		return false;
	}
	QueuedFrame q = std::move(m_queue.front());
	m_queue.pop_front();
	m_nQueuedSamples -= q.f->samples;
	SetEvent(m_hSpaceEvent);

	// Timestamp jump: offset the clock, like the original renderer.
	if (!m_segments.empty()) {
		const REFERENCE_TIME delta = q.rtStart - m_rtNextQueued;
		if ((delta > 200 || delta < -200) && OnClockOffset) {
			OnClockOffset(delta);
		}
	}
	m_rtNextQueued = q.rtStart + SamplesToTime(q.f->samples);

	Segment seg;
	seg.start    = m_end;
	seg.len      = q.f->samples;
	seg.channels = std::min<uint32_t>(q.f->channels, 16);
	seg.rtStart  = q.rtStart;
	memcpy(seg.elements, q.f->elements, sizeof(seg.elements));
	m_segments.push_back(seg);

	for (unsigned c = 0; c < 16; c++) {
		auto& v = m_in[c];
		const size_t old = v.size();
		v.resize(old + q.f->samples);
		for (uint32_t k = 0; k < q.f->samples; k++) {
			v[old + k] = (c < seg.channels) ? q.f->pcm[k][c] : 0.0f;
		}
	}
	m_end += q.f->samples;
	return true;
}

float CAtmosOutput::Sample(unsigned c, int64_t index) const
{
	if (index < (int64_t)m_base || index >= (int64_t)m_end) {
		return 0.0f;
	}
	return m_in[c][(size_t)(index - (int64_t)m_base)];
}

void CAtmosOutput::RenderPeriod()
{
	UINT32 availableDynamic = 0, frames = 0;
	if (FAILED(m_pStream->BeginUpdatingAudioObjects(&availableDynamic, &frames))) {
		return;
	}

	std::unique_lock<std::mutex> lock(m_mutex);

	// Fill the input buffer: it must cover the period plus the interpolation kernel.
	const double speed = m_bRunning ? m_dSpeed : 1.0;
	if (m_bRunning) {
		const double need = m_pos + frames * speed + KERNEL_HALF + 2;
		while ((double)m_end < need && PullFrame()) {
		}
	}

	// Find out the element kinds (LFE or object) as soon as data is available.
	const Segment* pFirst = SegmentAt((uint64_t)m_pos);
	if (!pFirst && !m_segments.empty()) {
		pFirst = &m_segments.front();
	}
	if (pFirst) {
		for (unsigned c = 0; c < pFirst->channels; c++) {
			const uint32_t kind = pFirst->elements[c].kind;
			if (m_pObjects[c] && m_ObjectKind[c] != kind) {
				m_pObjects[c]->SetEndOfStream(0);
				SafeRelease(m_pObjects[c]);
			}
			if (!m_pObjects[c]) {
				const AudioObjectType t = (kind == KIND_LFE) ? AudioObjectType_LowFrequency : AudioObjectType_Dynamic;
				if (SUCCEEDED(m_pStream->ActivateSpatialAudioObject(t, &m_pObjects[c]))) {
					m_ObjectKind[c] = kind;
				}
			}
		}
	}

	// Buffers of all active objects, zeroed.
	float* out[16] = {};
	for (unsigned c = 0; c < 16; c++) {
		if (m_pObjects[c]) {
			BYTE* b = nullptr;
			UINT32 len = 0;
			if (SUCCEEDED(m_pObjects[c]->GetBuffer(&b, &len)) && b) {
				memset(b, 0, len);
				out[c] = (float*)b;
			}
		}
	}

	if (m_bRunning && m_end > m_base) {
		if (const Segment* s = SegmentAt((uint64_t)m_pos)) {
			memcpy(m_State, s->elements, sizeof(m_State));
			m_bHaveState = true;
		}

		// Produce samples while the kernel stays within the buffer.
		UINT32 i = 0;
		for (; i < frames; i++) {
			const double p = m_pos;
			const int64_t ip = (int64_t)std::floor(p);
			if (ip + KERNEL_HALF >= (int64_t)m_end) {
				break; // underrun: the rest of the period is silent
			}
			const double frac = p - (double)ip;
			const unsigned phase = (unsigned)(frac * KERNEL_PHASES);
			if (phase == 0 && frac * KERNEL_PHASES < 1e-9) {
				// Integer position: exact copy (bit-exact at speed 1).
				for (unsigned c = 0; c < 16; c++) {
					if (out[c]) {
						out[c][i] = Sample(c, ip);
					}
				}
			} else {
				const float mu = (float)(frac * KERNEL_PHASES - phase);
				const float* h0 = SincKernel::Taps(phase);
				const float* h1 = SincKernel::Taps(phase + 1);
				for (unsigned c = 0; c < 16; c++) {
					if (!out[c]) {
						continue;
					}
					float acc = 0.0f;
					for (int k = 0; k < 2 * KERNEL_HALF; k++) {
						const float h = h0[k] + mu * (h1[k] - h0[k]);
						acc += h * Sample(c, ip - KERNEL_HALF + 1 + k);
					}
					out[c][i] = acc;
				}
			}
			m_pos += speed;
		}

		if (const Segment* s = SegmentAt((uint64_t)m_pos)) {
			m_rtCurrentRendered = s->rtStart + (REFERENCE_TIME)((m_pos - (double)s->start) * UNITS_PER_SEC / SAMPLE_RATE);
		}

		// Drop consumed input (keeping the kernel context).
		const int64_t keep = (int64_t)std::floor(m_pos) - KERNEL_HALF - 1;
		if (keep > (int64_t)m_base + 4096) {
			const size_t drop = (size_t)(keep - (int64_t)m_base);
			for (auto& v : m_in) {
				v.erase(v.begin(), v.begin() + drop);
			}
			m_base += drop;
			while (m_segments.size() > 1 && m_segments[1].start <= m_base) {
				m_segments.pop_front();
			}
		}
	}

	// Positions and gains at the start of the period.
	for (unsigned c = 0; c < 16; c++) {
		if (!m_pObjects[c]) {
			continue;
		}
		const Element& e = m_State[c];
		const float gain = (m_bHaveState && e.active) ? e.gain * m_fVolume : 0.0f;
		m_pObjects[c]->SetVolume(gain);
		if (m_ObjectKind[c] != KIND_LFE && m_bHaveState) {
			// DAMF: x left/right, y back/front, z floor/ceiling
			// Windows: x right, y up, z towards the back
			m_pObjects[c]->SetPosition(e.x, e.z, -e.y);
		}
	}

	lock.unlock();
	m_pStream->EndUpdatingAudioObjects();
}

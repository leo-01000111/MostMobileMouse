package eu.leongorecki.deskmouse.recorder

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothClass
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothHidDevice
import android.bluetooth.BluetoothHidDeviceAppSdpSettings
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.content.Context
import android.util.Log
import java.util.concurrent.Executors

private const val TAG = "DeskBt"

/**
 * The phone as a Bluetooth HID mouse (BluetoothHidDevice, Android 9+).
 * Report 1: buttons (3 bits + pad), X, Y (int16, relative), wheel, horizontal pan (int8, relative).
 * The host pairs with the phone like with any Bluetooth mouse; no software on the computer.
 */
@SuppressLint("MissingPermission") // callers check BLUETOOTH_CONNECT first
class BtMouse(private val ctx: Context) {
    interface Listener { fun onBtState(text: String) }

    var listener: Listener? = null
    private val adapter: BluetoothAdapter? = ctx.getSystemService(BluetoothManager::class.java)?.adapter
    private val exec = Executors.newSingleThreadExecutor()
    private var hid: BluetoothHidDevice? = null
    @Volatile var registered = false
        private set
    @Volatile var host: BluetoothDevice? = null
        private set
    private var buttons = 0

    val connected get() = host != null
    @Volatile private var wanted = false  // re-register if Android drops the HID app (e.g. after a failed connect)

    /** Paired computers only: a speaker or watch can't use the phone as a mouse. */
    val computers: List<BluetoothDevice>
        get() = adapter?.bondedDevices?.filter { it.bluetoothClass?.majorDeviceClass == BluetoothClass.Device.Major.COMPUTER }.orEmpty()

    private fun state(s: String) { Log.i(TAG, s); listener?.onBtState(s) }

    private val callback = object : BluetoothHidDevice.Callback() {
        override fun onAppStatusChanged(plugged: BluetoothDevice?, reg: Boolean) {
            registered = reg
            if (reg) {
                state("Bluetooth mouse ready${plugged?.let { " (last host ${it.name})" } ?: ""}")
            } else if (wanted) {
                state("Bluetooth mouse dropped by Android, registering again…")
                exec.execute { Thread.sleep(500); register() }
            } else {
                state("Bluetooth mouse off")
            }
        }

        override fun onConnectionStateChanged(device: BluetoothDevice, st: Int) {
            when (st) {
                BluetoothProfile.STATE_CONNECTED -> { host = device; state("connected to ${device.name}") }
                BluetoothProfile.STATE_DISCONNECTED -> { if (host == device) host = null; state("disconnected from ${device.name}") }
            }
        }

        // Hosts may ask for a report (rare for mice); answer with an idle one.
        override fun onGetReport(device: BluetoothDevice, type: Byte, id: Byte, bufferSize: Int) {
            hid?.replyReport(device, type, REPORT_ID, ByteArray(REPORT_LEN))
        }
    }

    /** Registers the HID app. Must be done before the host pairs, so the host sees a mouse. */
    fun start() {
        val a = adapter ?: return state("no Bluetooth adapter")
        if (!a.isEnabled) return state("Bluetooth is off")
        wanted = true
        a.getProfileProxy(ctx, object : BluetoothProfile.ServiceListener {
            override fun onServiceConnected(profile: Int, proxy: BluetoothProfile) {
                hid = proxy as BluetoothHidDevice
                register()
            }

            override fun onServiceDisconnected(profile: Int) { hid = null; registered = false; host = null; state("HID service gone") }
        }, BluetoothProfile.HID_DEVICE)
    }

    private fun register() {
        val h = hid ?: return
        if (registered) return
        val sdp = BluetoothHidDeviceAppSdpSettings(
            "Desk Mouse", "Phone used as a mouse", "MostMobileMouse", BluetoothHidDevice.SUBCLASS1_MOUSE, DESCRIPTOR,
        )
        if (!h.registerApp(sdp, null, null, exec, callback)) state("HID registerApp failed")
    }

    fun stop() {
        wanted = false
        hid?.let { h ->
            host?.let { h.disconnect(it) }
            h.unregisterApp()
            adapter?.closeProfileProxy(BluetoothProfile.HID_DEVICE, h)
        }
        hid = null; registered = false; host = null
    }

    /** Connects to an already paired computer (phone-initiated). */
    fun connect(device: BluetoothDevice): Boolean {
        val h = hid ?: return false
        state("connecting to ${device.name}…")
        return h.connect(device)
    }

    /** Relative motion in counts (+x right, +y down), wheel notches (+ up), pan notches (+ right). */
    fun move(dx: Int, dy: Int, wheel: Int = 0, pan: Int = 0) {
        var x = dx; var y = dy; var w = wheel; var p = pan
        // Split large values over several reports so nothing is clipped.
        do {
            val sx = x.coerceIn(-32767, 32767); val sy = y.coerceIn(-32767, 32767)
            val sw = w.coerceIn(-127, 127); val sp = p.coerceIn(-127, 127)
            send(buttons, sx, sy, sw, sp)
            x -= sx; y -= sy; w -= sw; p -= sp
        } while (x != 0 || y != 0 || w != 0 || p != 0)
    }

    /** Button bits: 1 = left, 2 = right, 4 = middle. */
    fun setButtons(b: Int) {
        if (b == buttons) return
        buttons = b
        send(buttons, 0, 0, 0, 0)
    }

    fun click(bit: Int) {
        setButtons(buttons or bit)
        setButtons(buttons and bit.inv())
    }

    private fun send(b: Int, x: Int, y: Int, w: Int, p: Int) {
        val h = hid ?: return
        val d = host ?: return
        val r = byteArrayOf(
            b.toByte(), (x and 0xFF).toByte(), (x shr 8).toByte(), (y and 0xFF).toByte(), (y shr 8).toByte(), w.toByte(), p.toByte(),
        )
        h.sendReport(d, REPORT_ID.toInt(), r)
    }

    companion object {
        const val REPORT_ID: Byte = 1
        const val REPORT_LEN = 7
        val DESCRIPTOR = byteArrayOf(
            0x05, 0x01,                    // Usage Page (Generic Desktop)
            0x09, 0x02,                    // Usage (Mouse)
            0xA1.toByte(), 0x01,           // Collection (Application)
            0x85.toByte(), 0x01,           //   Report ID (1)
            0x09, 0x01,                    //   Usage (Pointer)
            0xA1.toByte(), 0x00,           //   Collection (Physical)
            0x05, 0x09,                    //     Usage Page (Button)
            0x19, 0x01, 0x29, 0x03,        //     Usage Minimum (1), Maximum (3)
            0x15, 0x00, 0x25, 0x01,        //     Logical Minimum (0), Maximum (1)
            0x95.toByte(), 0x03, 0x75, 0x01, //   Report Count (3), Size (1)
            0x81.toByte(), 0x02,           //     Input (Data, Var, Abs)
            0x95.toByte(), 0x01, 0x75, 0x05, //   Report Count (1), Size (5)
            0x81.toByte(), 0x01,           //     Input (Const) padding
            0x05, 0x01,                    //     Usage Page (Generic Desktop)
            0x09, 0x30, 0x09, 0x31,        //     Usage (X), Usage (Y)
            0x16, 0x01, 0x80.toByte(),     //     Logical Minimum (-32767)
            0x26, 0xFF.toByte(), 0x7F,     //     Logical Maximum (32767)
            0x75, 0x10, 0x95.toByte(), 0x02, //   Report Size (16), Count (2)
            0x81.toByte(), 0x06,           //     Input (Data, Var, Rel)
            0x09, 0x38,                    //     Usage (Wheel)
            0x15, 0x81.toByte(), 0x25, 0x7F, //   Logical Minimum (-127), Maximum (127)
            0x75, 0x08, 0x95.toByte(), 0x01, //   Report Size (8), Count (1)
            0x81.toByte(), 0x06,           //     Input (Data, Var, Rel)
            0x05, 0x0C,                    //     Usage Page (Consumer)
            0x0A, 0x38, 0x02,              //     Usage (AC Pan)
            0x15, 0x81.toByte(), 0x25, 0x7F, //   Logical Minimum (-127), Maximum (127)
            0x75, 0x08, 0x95.toByte(), 0x01, //   Report Size (8), Count (1)
            0x81.toByte(), 0x06,           //     Input (Data, Var, Rel)
            0xC0.toByte(),                 //   End Collection
            0xC0.toByte(),                 // End Collection
        )
    }
}

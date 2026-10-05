package eu.leongorecki.deskmouse.recorder

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.content.Intent
import android.content.pm.PackageManager
import android.hardware.SensorManager
import android.hardware.camera2.CameraManager
import android.os.Bundle
import android.view.KeyEvent
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeDrawingPadding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import java.io.File

class MainActivity : ComponentActivity() {
    private lateinit var ctl: RecorderController
    private var hasCamera = false

    private val askCamera = registerForActivityResult(ActivityResultContracts.RequestPermission()) { ok ->
        hasCamera = ok
        if (ok) ctl.startLive() else ctl.status = "camera permission denied"
    }

    private var discoverAfterGrant = false
    private val askBt = registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { res ->
        if (res.values.all { it }) { startBt(); if (discoverAfterGrant) makeDiscoverable() }
        else ctl.btStatus = "Bluetooth permission denied"
        discoverAfterGrant = false
    }

    private val btPerms = arrayOf(Manifest.permission.BLUETOOTH_CONNECT, Manifest.permission.BLUETOOTH_ADVERTISE)

    /** Registers the phone as a Bluetooth mouse (asks for permissions the first time). */
    fun startBt() {
        if (!hasBtPerms()) { askBt.launch(btPerms); return }
        if (ctl.bt == null) ctl.bt = BtMouse(this).also { it.start() }
    }

    private fun hasBtPerms() = btPerms.all { checkSelfPermission(it) == PackageManager.PERMISSION_GRANTED }

    /** Registers the mouse, then lets a computer find the phone for pairing (system dialog, 120 s). */
    fun pairNew() {
        if (!hasBtPerms()) { discoverAfterGrant = true; askBt.launch(btPerms); return }
        startBt()
        makeDiscoverable()
    }

    private fun makeDiscoverable() {
        try {
            startActivity(Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE).putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, 120))
            ctl.btStatus = "discoverable for 120 s: on the PC, Add device → Bluetooth → pick this phone"
        } catch (e: SecurityException) {
            ctl.btStatus = "can't make the phone discoverable: ${e.message}"
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        val root = File(getExternalFilesDir(null), "recordings").apply { mkdirs() }
        ctl = RecorderController(
            getSystemService(CameraManager::class.java), getSystemService(SensorManager::class.java),
            root, BuildConfig.VERSION_NAME,
        )
        setContent { MaterialTheme(colorScheme = darkColorScheme()) { Screen(this, ctl, root) } }
    }

    override fun onResume() {
        super.onResume()
        hasCamera = checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED
        if (hasCamera) ctl.startLive() else askCamera.launch(Manifest.permission.CAMERA)
        // Keep the mouse registered whenever the app is open, so a computer pairing with the phone sees a mouse.
        if (hasBtPerms()) startBt()
    }

    override fun onPause() {
        ctl.stopLive()
        super.onPause()
    }

    override fun onDestroy() {
        ctl.bt?.stop()
        ctl.release()
        super.onDestroy()
    }

    override fun onKeyDown(keyCode: Int, event: KeyEvent): Boolean =
        ctl.onKey(keyCode, true, event.repeatCount > 0) || super.onKeyDown(keyCode, event)

    override fun onKeyUp(keyCode: Int, event: KeyEvent): Boolean =
        ctl.onKey(keyCode, false, false) || super.onKeyUp(keyCode, event)
}

@SuppressLint("MissingPermission") // bonded-device names: shown only after startBt() got BLUETOOTH_CONNECT
@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun Screen(act: MainActivity, c: RecorderController, root: File) {
    Box(Modifier.fillMaxSize().background(Color.Black)) {
        Column(
            Modifier.fillMaxSize().safeDrawingPadding().verticalScroll(rememberScrollState()).padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp),
        ) {
            Text("DeskMouse Recorder", fontSize = 22.sp, color = Color.White)
            Text(c.live.ifEmpty { "starting…" }, color = Color(0xFF9EE493), fontSize = 13.sp)
            if (c.status.isNotEmpty()) Text(c.status, color = Color(0xFFFF8A80))
            c.preview?.let {
                Image(it.asImageBitmap(), "camera preview", Modifier.fillMaxWidth().height(200.dp))
            }

            Label("Protocol")
            FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                for (p in PROTOCOLS) FilterChip(c.protocol == p, { c.protocol = p; c.seconds = p.seconds }, { Text(p.tag) })
            }
            Text(c.protocol.howTo, color = Color.LightGray, fontSize = 14.sp)
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                OutlinedTextField(
                    c.seconds.toString(), { v -> v.toIntOrNull()?.let { c.seconds = it.coerceIn(3, 1800) } },
                    label = { Text("Duration s") }, modifier = Modifier.weight(1f),
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
                )
                OutlinedTextField(
                    c.location, { c.location = it }, label = { Text("Place / desk surface") }, modifier = Modifier.weight(2f),
                )
            }

            Label("Camera")
            FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                for (ch in c.choices) FilterChip(c.choice == ch, { c.choice = ch; c.startLive() }, { Text(ch.label) })
            }
            FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                for (s in listOf(640 to 480, 1280 to 960)) FilterChip(c.size == s, { c.size = s; c.startLive() }, { Text("${s.first}×${s.second}") })
                for (f in listOf(60, 30)) FilterChip(c.fps == f, { c.fps = f; c.startLive() }, { Text("$f fps") })
            }
            Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                OutlinedTextField(
                    c.exposureMs.toString(), { v -> v.toFloatOrNull()?.let { c.exposureMs = it.coerceIn(0.1f, 16f) } },
                    label = { Text("Exposure ms") }, modifier = Modifier.weight(1f),
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Decimal),
                )
                Text("Raw frames", color = Color.White)
                Switch(c.raw, { c.raw = it })
            }

            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button({ c.record() }, enabled = c.phase == Phase.Idle && c.choice != null) { Text("Record (3 s countdown)") }
                OutlinedButton({ c.status = "saved ${c.saveDeviceCaps().name}" }) { Text("Save device caps") }
            }
            Button({ c.stream() }, enabled = c.phase == Phase.Idle && c.choice != null, modifier = Modifier.fillMaxWidth()) {
                Text("Mouse mode: stream to PC (USB or Wi-Fi)")
            }
            Label("Bluetooth mouse")
            Text(c.btStatus, color = Color(0xFF80D8FF), fontSize = 13.sp)
            FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                OutlinedButton({ act.startBt() }) { Text("Enable") }
                OutlinedButton({ act.pairNew() }) { Text("Pair new computer") }
                OutlinedButton({ c.btTestCircle() }) { Text("Test: draw circles") }
            }
            Button({ c.btMouse() }, enabled = c.phase == Phase.Idle && c.choice != null, modifier = Modifier.fillMaxWidth()) {
                Text("Bluetooth mouse (tracking on the phone)")
            }
            c.bt?.let { b ->
                if (b.registered) FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    val pcs = b.computers
                    if (pcs.isEmpty()) Text("No paired computer yet: use Pair new computer", color = Color.Gray, fontSize = 12.sp)
                    for (d in pcs) OutlinedButton({ b.connect(d) }) { Text("Connect ${d.name}") }
                }
            }
            Text("During a take: Vol-Up / Vol-Down = labels, both together = stop. Screen goes black and ignores touches.",
                color = Color.Gray, fontSize = 12.sp)
            if (c.lastSummary.isNotEmpty()) Text("Last: " + c.lastSummary, color = Color.White, fontSize = 13.sp)
            Text("Files: ${root.absolutePath}", color = Color.Gray, fontSize = 11.sp)
        }

        if (c.phase != Phase.Idle) {
            // Touch lock: face-down on a mouse pad the screen can register touches.
            Box(
                Modifier.fillMaxSize().background(Color.Black).pointerInput(Unit) {
                    awaitEachGesture { awaitFirstDown(requireUnconsumed = false).consume() }
                },
                contentAlignment = Alignment.Center,
            ) {
                Text("${c.phase} · ${c.remaining}\n${c.protocol.tag}", color = Color(0xFF444444), fontSize = 28.sp)
                if (c.phase == Phase.BtMouse) Text(c.status, color = Color(0xFF333333), fontSize = 11.sp,
                    modifier = Modifier.align(Alignment.BottomCenter).padding(16.dp))
            }
        }
    }
}

@Composable
private fun Label(s: String) = Text(s, color = Color.White, fontSize = 16.sp)

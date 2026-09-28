package eu.leongorecki.deskmouse.recorder

import android.Manifest
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

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        val root = File(getExternalFilesDir(null), "recordings").apply { mkdirs() }
        ctl = RecorderController(
            getSystemService(CameraManager::class.java), getSystemService(SensorManager::class.java),
            root, BuildConfig.VERSION_NAME,
        )
        setContent { MaterialTheme(colorScheme = darkColorScheme()) { Screen(ctl, root) } }
    }

    override fun onResume() {
        super.onResume()
        hasCamera = checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED
        if (hasCamera) ctl.startLive() else askCamera.launch(Manifest.permission.CAMERA)
    }

    override fun onPause() {
        ctl.stopLive()
        super.onPause()
    }

    override fun onDestroy() {
        ctl.release()
        super.onDestroy()
    }

    override fun onKeyDown(keyCode: Int, event: KeyEvent): Boolean =
        ctl.onKey(keyCode, true, event.repeatCount > 0) || super.onKeyDown(keyCode, event)

    override fun onKeyUp(keyCode: Int, event: KeyEvent): Boolean =
        ctl.onKey(keyCode, false, false) || super.onKeyUp(keyCode, event)
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun Screen(c: RecorderController, root: File) {
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
            }
        }
    }
}

@Composable
private fun Label(s: String) = Text(s, color = Color.White, fontSize = 16.sp)

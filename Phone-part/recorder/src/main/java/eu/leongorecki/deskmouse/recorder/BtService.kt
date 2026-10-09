package eu.leongorecki.deskmouse.recorder

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo

/**
 * Keeps the app at foreground-service importance while the Bluetooth mouse is on.
 * Android's HidDeviceService unregisters a HID app as soon as its process drops below that
 * (app in the background, Settings opened to manage pairing), and refuses to register it again
 * from the background. Without this the computer loses the mouse and can't reconnect.
 */
class BtService : Service() {
    override fun onBind(intent: Intent?) = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        val nm = getSystemService(NotificationManager::class.java)
        nm.createNotificationChannel(NotificationChannel(CHANNEL, "Bluetooth mouse", NotificationManager.IMPORTANCE_LOW))
        val open = PendingIntent.getActivity(this, 0, Intent(this, MainActivity::class.java), PendingIntent.FLAG_IMMUTABLE)
        val n = Notification.Builder(this, CHANNEL)
            .setSmallIcon(android.R.drawable.stat_sys_data_bluetooth)
            .setContentTitle("Desk Mouse")
            .setContentText("Bluetooth mouse is on")
            .setContentIntent(open)
            .setOngoing(true)
            .build()
        startForeground(1, n, ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE)
        return START_NOT_STICKY
    }

    companion object {
        private const val CHANNEL = "bt_mouse"
        fun start(ctx: Context) { ctx.startForegroundService(Intent(ctx, BtService::class.java)) }
        fun stop(ctx: Context) { ctx.stopService(Intent(ctx, BtService::class.java)) }
    }
}

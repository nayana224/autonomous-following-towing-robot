# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Responsive layout and window policy for the operator dashboard."""

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QApplication, QSizePolicy


class ResponsiveDashboardMixin:
    """Own all responsive layout, styling, and window sizing in one class."""

    DESIGN_WIDTH = 1256
    DESIGN_HEIGHT = 900
    WINDOW_WIDTH_RATIO = 0.92
    WINDOW_HEIGHT_RATIO = 0.90
    MINIMUM_WINDOW_WIDTH = 680
    MINIMUM_WINDOW_HEIGHT = 520

    def __init__(self):
        super().__init__()
        self._configure_window_flags()

    def _configure_window_flags(self):
        """Enable normal desktop window controls without forcing maximization."""
        self.setWindowFlags(
            Qt.Window
            | Qt.WindowTitleHint
            | Qt.WindowSystemMenuHint
            | Qt.WindowMinimizeButtonHint
            | Qt.WindowMaximizeButtonHint
            | Qt.WindowCloseButtonHint
        )
        self.setMinimumSize(
            self.MINIMUM_WINDOW_WIDTH,
            self.MINIMUM_WINDOW_HEIGHT,
        )
        self.setMaximumSize(16777215, 16777215)

    def _apply_responsive_layout(self):
        """Apply all dashboard and safety-page scaling exactly once per refresh."""
        screen = QApplication.primaryScreen()
        if screen is None:
            return

        available = screen.availableGeometry()
        width_scale = available.width() / self.DESIGN_WIDTH
        height_scale = available.height() / self.DESIGN_HEIGHT
        scale = max(0.68, min(1.0, width_scale, height_scale))

        self._configure_dashboard_layout(scale)
        self._configure_safety_layout(scale)
        self._apply_unified_styles(scale)

    def _configure_dashboard_layout(self, scale):
        """Configure the home dashboard without hidden fixed-size constraints."""
        outer_margin = max(10, round(24 * scale))
        main_spacing = max(8, round(14 * scale))
        top_spacing = max(12, round(22 * scale))
        content_margin = max(8, round(10 * scale))

        self.mainVerticalLayout.setContentsMargins(
            outer_margin,
            outer_margin,
            outer_margin,
            outer_margin,
        )
        self.mainVerticalLayout.setSpacing(main_spacing)
        self.mainVerticalLayout.setStretch(0, 1)
        self.mainVerticalLayout.setStretch(1, 1)

        self.topHorizontalLayout.setSpacing(top_spacing)
        self.topHorizontalLayout.setStretch(0, 1)
        self.topHorizontalLayout.setStretch(1, 1)
        self.statusVerticalLayout.setSpacing(max(7, round(10 * scale)))

        self.tabContentLayout.setContentsMargins(
            content_margin,
            content_margin,
            content_margin,
            content_margin,
        )
        self.homeLayout.setSpacing(max(8, round(10 * scale)))
        self.homeLayout.setStretch(0, 1)
        self.homeLayout.setStretch(1, 4)
        self.homeButtonsLayout.setSpacing(max(8, round(10 * scale)))

        self.joystick_widget.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )
        self.joystick_widget.setMinimumSize(
            max(150, round(170 * scale)),
            max(150, round(170 * scale)),
        )
        self.joystick_widget.setMaximumSize(16777215, 16777215)

    def _configure_safety_layout(self, scale):
        """Give safety text and the release button guaranteed visible space."""
        if not hasattr(self, "page_safety_stop"):
            return

        self.label_safety_image.setMinimumSize(0, 0)
        self.label_safety_image.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )
        self.page_safety_stop.safety_image_card.setMinimumSize(0, 0)
        self.page_safety_stop.safety_info_card.setMinimumSize(0, 0)

        root_layout = self.page_safety_stop.safetyRootLayout
        root_layout.setStretch(0, 1)
        root_layout.setStretch(1, 1)
        root_layout.setSpacing(max(8, round(16 * scale)))
        root_margin = max(7, round(14 * scale))
        root_layout.setContentsMargins(
            root_margin,
            root_margin,
            root_margin,
            root_margin,
        )

        image_margin = max(5, round(9 * scale))
        self.page_safety_stop.safetyImageLayout.setContentsMargins(
            image_margin,
            image_margin,
            image_margin,
            image_margin,
        )

        info_layout = self.page_safety_stop.safetyInfoLayout
        info_margin = max(8, round(14 * scale))
        info_layout.setContentsMargins(
            info_margin,
            info_margin,
            info_margin,
            info_margin,
        )
        info_layout.setSpacing(max(6, round(9 * scale)))

        spacer_item = info_layout.itemAt(4)
        if spacer_item is not None and spacer_item.spacerItem() is not None:
            spacer_item.spacerItem().changeSize(
                0,
                0,
                QSizePolicy.Minimum,
                QSizePolicy.Fixed,
            )

        self.label_safety_title.setSizePolicy(
            QSizePolicy.Preferred,
            QSizePolicy.Minimum,
        )
        self.label_safety_message.setSizePolicy(
            QSizePolicy.Preferred,
            QSizePolicy.Minimum,
        )
        self.label_safety_detail.setSizePolicy(
            QSizePolicy.Preferred,
            QSizePolicy.MinimumExpanding,
        )
        self.label_safety_title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.label_safety_message.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.label_safety_detail.setAlignment(Qt.AlignLeft | Qt.AlignTop)

    def _apply_unified_styles(self, scale):
        """Replace responsive overrides instead of repeatedly appending them."""
        if not hasattr(self, "_base_dashboard_style"):
            self._base_dashboard_style = self.styleSheet()
        if not hasattr(self, "_base_safety_style"):
            self._base_safety_style = self.page_safety_stop.styleSheet()

        label_font = max(13, round(17 * scale))
        main_button_font = max(30, round(44 * scale))
        checkbox_font = max(21, round(28 * scale))
        button_padding = max(8, round(14 * scale))
        card_padding_v = max(9, round(14 * scale))
        card_padding_h = max(12, round(18 * scale))
        indicator_size = max(25, round(32 * scale))

        dashboard_override = f"""
            QLabel {{
                font-size: {label_font}pt;
            }}
            QLabel#label_linear_vel,
            QLabel#label_angular_vel,
            QLabel#label_robot_status {{
                padding: {card_padding_v}px {card_padding_h}px;
            }}
            QTabBar::tab {{
                padding: {max(5, round(7 * scale))}px {max(10, round(15 * scale))}px;
            }}
            QCheckBox#save_path_check {{
                padding: {card_padding_v}px {card_padding_h}px;
                font-size: {checkbox_font}px;
                spacing: {max(8, round(13 * scale))}px;
            }}
            QCheckBox#save_path_check::indicator {{
                width: {indicator_size}px;
                height: {indicator_size}px;
            }}
            QPushButton {{
                padding: {button_padding}px;
            }}
            QPushButton#btn_follow,
            QPushButton#btn_autonomous {{
                font-size: {main_button_font}px;
            }}
        """
        self.setStyleSheet(self._base_dashboard_style + dashboard_override)

        title_font = max(18, round(26 * scale))
        message_font = max(14, round(17 * scale))
        detail_font = max(11, round(13 * scale))
        badge_font = max(11, round(13 * scale))
        release_font = max(15, round(19 * scale))
        release_height = max(60, round(76 * scale))
        detail_padding = max(7, round(10 * scale))

        safety_override = f"""
            QLabel#label_safety_badge {{
                padding: {max(5, round(6 * scale))}px {max(9, round(10 * scale))}px;
                font-size: {badge_font}pt;
            }}
            QLabel#label_safety_title {{
                font-size: {title_font}pt;
            }}
            QLabel#label_safety_message {{
                font-size: {message_font}pt;
            }}
            QLabel#label_safety_detail {{
                padding: {detail_padding}px;
                font-size: {detail_font}pt;
            }}
            QLabel#label_safety_image {{
                padding: {max(5, round(8 * scale))}px;
                font-size: {max(11, round(14 * scale))}pt;
            }}
            QPushButton#btn_clear_safety_stop {{
                min-height: {release_height}px;
                padding: {max(7, round(9 * scale))}px {max(10, round(13 * scale))}px;
                font-size: {release_font}pt;
            }}
        """
        self.page_safety_stop.setStyleSheet(
            self._base_safety_style + safety_override
        )

        self.label_safety_title.adjustSize()
        self.label_safety_message.adjustSize()
        self.label_safety_detail.adjustSize()
        self.page_safety_stop.safetyInfoLayout.invalidate()
        self.page_safety_stop.safetyInfoLayout.activate()

    def show_for_available_screen(self):
        """Show a resizable window, then fit its decorated frame to the desktop."""
        self._configure_window_flags()
        self.showNormal()
        QTimer.singleShot(0, self._fit_window_to_available_screen)

    def _fit_window_to_available_screen(self):
        """Fit the normal window below full-screen size and center its frame."""
        screen = self.screen() or QApplication.primaryScreen()
        if screen is None:
            return

        available = screen.availableGeometry()
        target_width = min(
            self.DESIGN_WIDTH,
            max(
                self.MINIMUM_WINDOW_WIDTH,
                int(available.width() * self.WINDOW_WIDTH_RATIO),
            ),
        )
        target_height = min(
            self.DESIGN_HEIGHT,
            max(
                self.MINIMUM_WINDOW_HEIGHT,
                int(available.height() * self.WINDOW_HEIGHT_RATIO),
            ),
        )

        self.resize(target_width, target_height)
        frame = self.frameGeometry()
        frame.moveCenter(available.center())
        self.move(frame.topLeft())

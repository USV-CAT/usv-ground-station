//! A file picker drawn by the station itself.
//!
//! In the Deck's Game Mode there is no desktop to lend a file dialog, so the
//! station has its own: big rows for a thumb, folders first, only the files
//! that can be opened.

use std::path::{Path, PathBuf};

use egui::{Context, RichText};

use crate::ui::theme;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Purpose {
    OpenMission,
    SaveMission,
    ExportWaypoints,
    ExportSoundings,
    ExportGrid,
    LoadChart,
}

pub struct FileDialog {
    pub purpose: Purpose,
    pub title: String,
    pub dir: PathBuf,
    pub name: String,
    pub save: bool,
    pub extensions: Vec<&'static str>,
    entries: Vec<(String, bool)>,
    listed: Option<PathBuf>,
}

pub enum Outcome {
    Chosen(PathBuf),
    Cancelled,
}

impl FileDialog {
    pub fn open(purpose: Purpose, title: &str, dir: PathBuf, extensions: &[&'static str]) -> Self {
        FileDialog {
            purpose,
            title: title.into(),
            dir,
            name: String::new(),
            save: false,
            extensions: extensions.to_vec(),
            entries: Vec::new(),
            listed: None,
        }
    }

    pub fn save(purpose: Purpose, title: &str, dir: PathBuf, name: String, extensions: &[&'static str]) -> Self {
        FileDialog { save: true, name, ..FileDialog::open(purpose, title, dir, extensions) }
    }

    fn list(&mut self) {
        if self.listed.as_deref() == Some(self.dir.as_path()) {
            return;
        }
        let _ = std::fs::create_dir_all(&self.dir);
        let mut entries: Vec<(String, bool)> = std::fs::read_dir(&self.dir)
            .map(|rd| {
                rd.filter_map(|e| e.ok())
                    .filter_map(|e| {
                        let name = e.file_name().to_string_lossy().to_string();
                        if name.starts_with('.') {
                            return None;
                        }
                        let is_dir = e.path().is_dir();
                        let wanted = is_dir || self.extensions.is_empty() || self.extensions.iter().any(|x| name.to_lowercase().ends_with(x));
                        wanted.then_some((name, is_dir))
                    })
                    .collect()
            })
            .unwrap_or_default();
        entries.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.to_lowercase().cmp(&b.0.to_lowercase())));
        self.entries = entries;
        self.listed = Some(self.dir.clone());
    }

    /// Draw the picker; Some when the operator has decided.
    pub fn show(&mut self, ctx: &Context) -> Option<Outcome> {
        self.list();
        let mut outcome = None;
        let mut go_to: Option<PathBuf> = None;
        egui::Modal::new(egui::Id::new("file-dialog")).show(ctx, |ui| {
            ui.set_width(560.0f32.min(ctx.content_rect().width() - 40.0));
            ui.label(RichText::new(&self.title).font(theme::bold(16.0)).color(theme::ACCENT));
            ui.horizontal(|ui| {
                if ui.button("⬆ Up").clicked() {
                    if let Some(parent) = self.dir.parent() {
                        go_to = Some(parent.to_path_buf());
                    }
                }
                if ui.button("⌂ Home").clicked() {
                    if let Some(home) = std::env::var_os("HOME") {
                        go_to = Some(PathBuf::from(home));
                    }
                }
                ui.label(RichText::new(self.dir.display().to_string()).color(theme::TEXT_DIM).small());
            });
            ui.separator();
            egui::ScrollArea::vertical().max_height(ctx.content_rect().height() * 0.5).show(ui, |ui| {
                ui.set_width(ui.available_width());
                if self.entries.is_empty() {
                    ui.label(RichText::new("Nothing here").color(theme::TEXT_DIM));
                }
                for (name, is_dir) in self.entries.clone() {
                    let text = if is_dir { format!("📁  {name}") } else { format!("📄  {name}") };
                    let selected = !is_dir && self.name == name;
                    let r = ui.add_sized([ui.available_width(), 34.0], egui::Button::selectable(selected, text));
                    if r.clicked() {
                        if is_dir {
                            go_to = Some(self.dir.join(&name));
                        } else {
                            self.name = name.clone();
                        }
                    }
                    if r.double_clicked() && !is_dir {
                        outcome = Some(Outcome::Chosen(self.dir.join(&name)));
                    }
                }
            });
            ui.separator();
            ui.horizontal(|ui| {
                ui.label(if self.save { "Save as" } else { "File" });
                ui.add(egui::TextEdit::singleline(&mut self.name).desired_width(320.0));
            });
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                if ui.add_sized([110.0, 36.0], egui::Button::new("Cancel")).clicked() {
                    outcome = Some(Outcome::Cancelled);
                }
                let label = if self.save { "Save" } else { "Open" };
                let ready = !self.name.trim().is_empty();
                if ui.add_enabled(ready, egui::Button::new(RichText::new(label).strong()).min_size(egui::vec2(110.0, 36.0))).clicked() {
                    let name = self.name.trim();
                    let path = if Path::new(name).is_absolute() { PathBuf::from(name) } else { self.dir.join(name) };
                    outcome = Some(Outcome::Chosen(path));
                }
            });
        });
        if let Some(dir) = go_to {
            self.dir = dir;
        }
        outcome
    }
}
